#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace passive_stereo
{

class StereoRectifyNode : public rclcpp::Node
{
public:
    explicit StereoRectifyNode(const rclcpp::NodeOptions & options)
    : Node("stereo_rectify_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Iniciando Stereo Rectify (Info Callback separado)...");

        // QoS padrão para sensores (Best Effort é comum para vídeo)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

        // 1. Publishers (Saída Retificada - Zero Copy via Unique Ptr)
        pub_left_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_rect", qos);
        pub_right_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_rect", qos);

        // 2. Subscribers para CameraInfo (Callbacks separados e simples)
        // Não usamos message_filters aqui. Apenas pegamos a calibração uma vez.
        sub_info_left_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "left/camera_info", qos,
            std::bind(&StereoRectifyNode::infoLeftCallback, this, std::placeholders::_1));

        sub_info_right_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "right/camera_info", qos,
            std::bind(&StereoRectifyNode::infoRightCallback, this, std::placeholders::_1));

        // 3. Subscribers para Imagens (Message Filters para Sincronização Estéreo)
        sub_img_left_.subscribe(this, "left/image_raw", custom_qos_profile);
        sub_img_right_.subscribe(this, "right/image_raw", custom_qos_profile);

        // Sincroniza apenas as duas imagens
        // ExactTime é ideal para câmeras estéreo hardware-synced.
        // Se houver jitter no timestamp, use ApproximateTime.
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_img_left_, sub_img_right_
        );

        sync_->registerCallback(std::bind(&StereoRectifyNode::stereoImageCallback, this, 
            std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Estado interno
    std::mutex map_mutex_; // Protege acesso aos mapas caso use MultiThreadedExecutor
    bool valid_left_map_ = false;
    bool valid_right_map_ = false;

    // Mapas de retificação (Lookup Tables)
    cv::Mat l_map1_, l_map2_;
    cv::Mat r_map1_, r_map2_;

    // --- Callbacks de Camera Info ---
    
    void infoLeftCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (valid_left_map_) return; // Já calculamos, ignora atualizações (ou remova para permitir recalibração dinâmica)

        initMap(msg, l_map1_, l_map2_);
        valid_left_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Calibração ESQUERDA recebida e mapas gerados.");
    }

    void infoRightCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (valid_right_map_) return; 

        initMap(msg, r_map1_, r_map2_);
        valid_right_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Calibração DIREITA recebida e mapas gerados.");
    }

    // Função auxiliar para gerar mapas a partir dos dados intrínsecos
    void initMap(const sensor_msgs::msg::CameraInfo::SharedPtr& info, cv::Mat& map1, cv::Mat& map2)
    {
        cv::Mat K = cv::Mat(3, 3, CV_64F, (void*)info->k.data());
        cv::Mat D = cv::Mat(1, 5, CV_64F, (void*)info->d.data());
        cv::Mat R = cv::Mat(3, 3, CV_64F, (void*)info->r.data());
        cv::Mat P = cv::Mat(3, 4, CV_64F, (void*)info->p.data());
        cv::Size img_size(info->width, info->height);

        // CV_16SC2 é mais rápido para o remap (ponto fixo), CV_32FC1 é mais preciso
        cv::initUndistortRectifyMap(K, D, R, P, img_size, CV_16SC2, map1, map2);
    }

    // --- Callback Principal Sincronizado (Só Imagens) ---

    void stereoImageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& l_img_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& r_img_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Recebidas imagens sincronizadas para retificação.");
        // Verifica se temos calibração antes de processar
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!valid_left_map_ || !valid_right_map_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Aguardando CameraInfo para gerar mapas de retificação...");
                return;
            }
        }

        // Processamento (pode ser paralelizado com OpenMP se desejar)
        auto rect_left = rectifyImage(l_img_msg, l_map1_, l_map2_);
        auto rect_right = rectifyImage(r_img_msg, r_map1_, r_map2_);

        // Publica usando move semantics para IPC
        if (rect_left) pub_left_->publish(std::move(rect_left));
        if (rect_right) pub_right_->publish(std::move(rect_right));
    }

    std::unique_ptr<sensor_msgs::msg::Image> rectifyImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& in_msg,
        const cv::Mat& map1, const cv::Mat& map2)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            // toCvShare é essencial para ZERO-COPY na leitura
            cv_ptr = cv_bridge::toCvShare(in_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro cv_bridge: %s", e.what());
            return nullptr;
        }

        auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
        out_msg->header = in_msg->header;
        out_msg->height = in_msg->height;
        out_msg->width = in_msg->width;
        out_msg->encoding = in_msg->encoding;
        out_msg->step = in_msg->step;
        out_msg->data.resize(out_msg->step * out_msg->height);

        // Cria matriz OpenCV apontando para o buffer da mensagem de saída recém criada
        cv::Mat img_out(out_msg->height, out_msg->width, cv_ptr->image.type(), out_msg->data.data(), out_msg->step);

        // Remap usando os mapas cacheados
        // Se usar maps CV_16SC2, é muito rápido usando instruções SIMD da CPU
        cv::remap(cv_ptr->image, img_out, map1, map2, cv::INTER_LINEAR);

        return out_msg;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_left_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_right_;

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_left_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_right_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} // namespace passive_stereo

RCLCPP_COMPONENTS_REGISTER_NODE(passive_stereo::StereoRectifyNode)