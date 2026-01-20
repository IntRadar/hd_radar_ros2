#include "hd_radar_driver/modules/hd_radar_heat.hpp"

void HdRadarHeat::BindCallback(std::function<void()> func)
{
    PubCallback = std::bind(func);
}

void HdRadarHeat::ParseHeat(char * buffer, size_t buf_len)
{
    memset(&msg_heat_, 0, sizeof(udp_msg_heat_t));
    memcpy(&msg_heat_, buffer, static_cast<size_t>(buf_len));
    
    uint16_t udp_idx = msg_heat_.heat_header.udp_idx;
    
    //Backward compatibility with version 1.2
    if ((msg_heat_.pre_header.version&0x0F) < 3) udp_idx -= 1;

    if (heat_full_.size() != msg_heat_.heat_header.udp_total*UDP_PAYLOAD_SIZE) {
        heat_full_.resize(msg_heat_.heat_header.udp_total*UDP_PAYLOAD_SIZE);
        std::fill(heat_full_.begin(), heat_full_.end(), 0);
    }

    RCLCPP_DEBUG(node_->get_logger(), "HEAT: frame_current %d curr_frame %ld",
    msg_heat_.heat_header.frame_cnt, heat_cur_frame_);
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: idx %d total %d",
    msg_heat_.heat_header.udp_idx, msg_heat_.heat_header.udp_total);
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: map_h_size %d map_v_size %d",
    msg_heat_.heat_header.map_h_size, msg_heat_.heat_header.map_v_size);

    if (msg_heat_.heat_header.frame_cnt != heat_cur_frame_) {
        // Publish when udp dgrams dropped
        // if (heat_cur_subframe_ != 0) PublishHeat();  
        
         // NTP sync check
        uint64_t high_bytes, time_stamp_radar;
        high_bytes = static_cast<uint64_t>(msg_heat_.heat_header.tv_usec_msb);
        time_stamp_radar = (high_bytes << 32 |
                        static_cast<uint64_t>(msg_heat_.heat_header.tv_usec_lsb));

        uint64_t delta_time = ((uint64_t)(node_->now().nanoseconds()/1000) -
                                time_stamp_radar);
        // RCLCPP_INFO(node_->get_logger(), "HEAT: delta: %ld", delta_time);

        // Check if delta_time greater then 1 sec
        if (delta_time > 1000000) {
            msg_heat_hd_.header.stamp = node_->now();
            msg_img_header_.stamp = node_->now();
            mtx_->lock();
            *ntp_sync_ = false;
            mtx_->unlock();
        }
        else {
            msg_heat_hd_.header.stamp = rclcpp::Time(time_stamp_radar * 
                                    static_cast<uint64_t>(1e3));
            msg_img_header_.stamp = rclcpp::Time(time_stamp_radar * 
                                    static_cast<uint64_t>(1e3));
            mtx_->lock();
            *ntp_sync_ = true;
            mtx_->unlock();
        }
        
        heat_cur_subframe_ = 0;
        heat_cur_frame_ = msg_heat_.heat_header.frame_cnt;
    }

    memcpy(&heat_full_[(udp_idx) * UDP_PAYLOAD_SIZE/sizeof(uint8_t)],
                        &msg_heat_.data[0], UDP_PAYLOAD_SIZE);
    heat_cur_subframe_++;

    if ((udp_idx + 1) == msg_heat_.heat_header.udp_total) {
        PublishHeat();
        heat_cur_subframe_ = 0;
    }
}

void HdRadarHeat::FillHeat()
{
    //Filling native heat map message
    msg_heat_hd_.h_size = msg_heat_.heat_header.map_h_size;
    msg_heat_hd_.v_size = msg_heat_.heat_header.map_v_size;
    msg_heat_hd_.array_size = heat_full_.size();
    msg_heat_hd_.heat_data = heat_full_;
    msg_heat_hd_.header.frame_id = frame_id_;

    cv::Mat refined_heat = cv::Mat(msg_heat_.heat_header.map_h_size, 
        msg_heat_.heat_header.map_v_size, CV_8UC1, 0.0);
    cv::Mat img_cart = cv::Mat(512, 1024, CV_8UC1, 0.0);
    cv::Mat colour_map;
    
    //Calculate pixels values
    for (size_t i = 0; i < msg_heat_.heat_header.map_h_size; ++i) {
        for (size_t j = 0; j < msg_heat_.heat_header.map_v_size; ++j) {
        double pix_val = (double)heat_full_[i*msg_heat_.heat_header.map_h_size + j];
        pix_val = std::pow(pix_val, img_pow_);
        double q = (std::pow(img_max_val_, img_pow_) -
                    std::pow(img_min_val_, img_pow_))/256.0;
        uint32_t pixel = (uint32_t)(pix_val/q);
        refined_heat.at<uint8_t>(i, j) = pixel < 255 ? pixel : 255;
        }
    }

    // RV map 
    if (msg_heat_.heat_header.map_type == 0) {
        //FFT shift
        int cx = refined_heat.cols;
        int cy = refined_heat.rows/2;

        cv::Mat tmp;
        cv::Mat q0(refined_heat, cv::Rect(0, 0, cx, cy));
        cv::Mat q1(refined_heat, cv::Rect(0, cy, cx, cy));

        q0.copyTo(tmp);
        q1.copyTo(q0);
        tmp.copyTo(q1);

        //Apply colormap
        applyColorMap(refined_heat, colour_map, cv::COLORMAP_JET);
    }
    //RA map
    else if (msg_heat_.heat_header.map_type == 1) {
        // Convert image to cartesian
        ImgPolarToCart(refined_heat, img_cart);

        //Apply colormap and flip vertically
        applyColorMap(img_cart, colour_map, cv::COLORMAP_JET);
        cv::flip(colour_map, colour_map, 0);
    }

    // cv::medianBlur(colour_map, colour_map, 5);
    
    //Filling heat map image message
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: img_min_val_: %f, img_max_val_: %f",
                img_min_val_, img_max_val_);
    msg_img_header_.frame_id = frame_id_;
    msg_img_ = cv_bridge::CvImage(msg_img_header_, "bgr8", 
                                colour_map).toImageMsg();
}

void HdRadarHeat::PublishHeat()
{
    // Heat fields initialization and publish message
    FillHeat();
    PubCallback();
}

void HdRadarHeat::ImgPolarToCart(cv::Mat &src, cv::Mat &dst)
{
    int azm_pad = src.rows;
    int rng_pad = src.cols;
    int img_size_x = dst.cols;
    int img_size_y = dst.rows;
    double dx, dy;
    double x_0, y_0, x_1, y_1, x_2, y_2, x_3, y_3;
    uint px_0, py_0, px_1, py_1, px_2, py_2, px_3, py_3;
    std::vector<double> thetas;
    std::vector<double> ranges;
    std::vector<std::vector<double>> rho_grid, phi_grid;
    std::vector<cv::Point> pnts;
    
    //Pixel step
    dx = (double)img_size_x/(2*img_max_rng_);
    dy = (double)img_size_y/img_max_rng_;

    //Calculate resolutions
    double azm_res = 2.0/azm_pad; 
    double rng_res = img_max_rng_/rng_pad; 
    double sin_val = -1.0;

    //Fill ranges, thetas
    for (int i = 0; i < azm_pad; i++) {
        thetas.push_back(asinf(sin_val));
        sin_val += azm_res;
    }

    for (int i = 0; i < rng_pad; i++) ranges.push_back(i*rng_res);

    //Erase first azimuth value for symetric map
    thetas.erase(thetas.begin());

    //Calculate meshgrids
    Meshgrid(thetas, ranges, phi_grid, rho_grid);

    for (int i = 0; i < (int)(rho_grid.size() - 1); ++i) {
        for (int j = 0; j < (int)(rho_grid[0].size() - 1); ++j) {
            //Convert meshgrids to cartesian
            x_0 = rho_grid[i][j] * sinf(phi_grid[i][j]); 
            y_0 = rho_grid[i][j] * cosf(phi_grid[i][j]);
            x_1 = rho_grid[i][j+1] * sinf(phi_grid[i][j+1]); 
            y_1 = rho_grid[i][j+1] * cosf(phi_grid[i][j+1]);
            x_2 = rho_grid[i+1][j+1] * sinf(phi_grid[i+1][j+1]); 
            y_2 = rho_grid[i+1][j+1] * cosf(phi_grid[i+1][j+1]);
            x_3 = rho_grid[i+1][j] * sinf(phi_grid[i+1][j]); 
            y_3 = rho_grid[i+1][j] * cosf(phi_grid[i+1][j]);

            // Draw polygons from meshgrids
            px_0 = (uint)(x_0*dx + img_size_x/2);
            py_0 = (uint)(y_0*dy);
            px_1 = (uint)(x_1*dx + img_size_x/2);
            py_1 = (uint)(y_1*dy);
            px_2 = (uint)(x_2*dx + img_size_x/2);
            py_2 = (uint)(y_2*dy);
            px_3 = (uint)(x_3*dx + img_size_x/2);
            py_3 = (uint)(y_3*dy);
            pnts.push_back(cv::Point(px_0, py_0));
            pnts.push_back(cv::Point(px_1, py_1));
            pnts.push_back(cv::Point(px_2, py_2));
            pnts.push_back(cv::Point(px_3, py_3));
            cv::fillConvexPoly(dst, pnts, src.at<uint8_t>(i, j)); 
            pnts.clear();
        }
    }
}

void HdRadarHeat::Meshgrid(const std::vector<double>& x_coords, 
              const std::vector<double>& y_coords,
              std::vector<std::vector<double>> &X, 
              std::vector<std::vector<double>> &Y)
{
    
    int num_x = x_coords.size();
    int num_y = y_coords.size();

    // Resize X and Y to hold the grid
    X.resize(num_y, std::vector<double>(num_x));
    Y.resize(num_y, std::vector<double>(num_x));
 
    for (int i = 0; i < num_y; ++i) {
        for (int j = 0; j < num_x; ++j) {
            // Populate X
            X[i][j] = x_coords[j];
            // Populate Y
            Y[i][j] = y_coords[i]; 
        }
    }
}

void HdRadarHeat::InitParams(std::string * frame_id, bool * ntp_sync,
                            std::mutex * mtx,  rclcpp::Node * node, 
                            double * img_min_val, double * img_max_val,
                            double * img_pow, double * img_max_rng) 
{
        frame_id_ = *frame_id;
        ntp_sync_ = ntp_sync;
        mtx_ = mtx;
        node_ = node;
        img_min_val_ = *img_min_val;
        img_max_val_ = *img_max_val;
        img_pow_ = *img_pow;
        img_max_rng_ = *img_max_rng;
}

HdRadarHeat::HdRadarHeat()
{
}

HdRadarHeat::~HdRadarHeat()
{
}
