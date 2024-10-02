#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

#define _MY_STU_ID_ 2204112913
    
const Vector2d _X0_(_MY_STU_ID_ % 827, _MY_STU_ID_ % 1709);
const double _LAMBDA_ = 0.5;
const double _DELTA_ = 0.01;

int main(int argc, char * argv[]){
    auto console_sink =std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log", true);
    spdlog::logger Optimizer_logger("Optimizer", {console_sink, file_sink});
    Optimizer_logger.set_level(spdlog::level::debug);
    // Optimizer_logger.set_pattern("[%n] [%l] %v"); 
    // 我不必这样做

    Matrix2d Hesse = 2 * Matrix2d::Identity();
    Matrix2d Hesse_inv = Hesse.inverse();

    Vector2d x_k = _X0_;
    Vector2d x_kk = _X0_;

    do{
        Optimizer_logger.debug("({}, {})", x_kk[0], x_kk[1]);
        x_k = x_kk;
        x_kk = x_k - _LAMBDA_ * Hesse_inv * Vector2d(2 * x_k[0], 2 * x_k[1]);
        
    } while ((x_k-x_kk).norm() >= _DELTA_);
    Optimizer_logger.info("{}", x_k.norm() * x_k.norm());
    
    return 0;
}