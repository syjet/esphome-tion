#include <algorithm>
#include <cmath>
#include "log.h"
#include "pi_controller.h"

namespace dentra {
namespace tion {
namespace auto_co2 {

static const char *const TAG = "pi_controller";
static const char *const PI_CONTROLLER_VERSION = "v1.4.3";

void PIController::reset(float kp, float ti, int db) {
  this->kp_ = kp;
  this->ti_ = ti;
  this->db_ = db;
  this->reset();
}

void PIController::reset(float kp, float ti, int db, float min, float max) {
  this->kp_ = kp;
  this->ti_ = ti;
  this->db_ = db;
  this->v_oa_min_ = min;
  this->v_oa_max_ = max;
  this->reset();
}

float PIController::update(int setpoint, int current) {
  if (this->kp_ <= 0.0f || this->ti_ <= 0.0f || std::isnan(this->v_oa_min_) || std::isnan(this->v_oa_max_)) {
    TION_LOGW(TAG, "Incorrect parameters: kp=%f, ti=%f, min=%f, max=%f", this->kp_, this->ti_, this->v_oa_min_, this->v_oa_max_);
    return 0.0f;
  }

  // 7: error from setpoint [ppm]
  const auto e = setpoint - current;

  // 8: error from setpoint, including dead band [ppm]
  const auto e_db =  //
      /**/ current < setpoint - this->db_ ? e - this->db_ :
      /**/ current > setpoint + this->db_ ? e + this->db_
                                          : 0;

  // Базовый интеграл, соответствующий минимальному притоку v_oa_min_ при e_db > 0
  const float base_min_ib = -this->ti_ * (this->v_oa_min_ / this->kp_);
  // Ниже этого e_db ветка v_oa_c < v_oa_min_ недостижима при ib <= 0
  const float e_db_floor = -this->v_oa_min_ / this->kp_;

  // 9: integral error [min.ppm-CO2]
  const float dt_s = std::clamp(this->dt_s_(), 0.0f, 120.0f);
  float i = this->ib_;
  if (e_db != 0) {
    i += (dt_s / 60.0f) * e_db;
  } else {
    const float deadband_target_ib = -this->ti_ * (std::max(0.0f, this->v_oa_min_ - 0.3f) / this->kp_);
    if (i < deadband_target_ib) {
      const float decay_tau = std::max(this->ti_ * 2.0f, 1.0f);
      const float alpha = std::min((dt_s / 60.0f) / decay_tau, 1.0f);
      i += alpha * (deadband_target_ib - i);
    }
  }

  // Динамическое ограничение интеграла при e_db < 0:
  // интегральный добавок сверх v_oa_min_ не превышает 2 * kp * |e_db|,
  // чтобы при небольшом превышении CO2 бризер не накручивал максимум скорости
  if (e_db < 0) {
    const float max_i_flow = std::min(this->v_oa_max_, this->v_oa_min_ + 2.0f * this->kp_ * (-e_db));
    const float dynamic_ib_floor = -this->ti_ * (max_i_flow / this->kp_);
    i = std::max(i, dynamic_ib_floor);
  }

  // 10: candidate outdoor airflow rate [m3/h]
  const float v_oa_c = -this->kp_ * (e_db + (i / this->ti_));

  // 11: integral error with anti-integral windup [min.ppm-CO2]
  if (e_db > 0 && v_oa_c < this->v_oa_min_) {
    const float windup_min_ib = -this->ti_ * (std::max(static_cast<float>(e_db), e_db_floor) + (this->v_oa_min_ / this->kp_));
    this->ib_ = std::max(windup_min_ib, base_min_ib);
  } else if (v_oa_c > this->v_oa_max_) {
    const float windup_max_ib = -this->ti_ * (e_db + (this->v_oa_max_ / this->kp_));
    this->ib_ = std::min(windup_max_ib, 0.0f);
  } else {
    this->ib_ = i;
  }

  // 12: outdoor airflow rate [m3/h]
  const float v_oa = -this->kp_ * (e_db + (this->ib_ / this->ti_));
  const float v_oa_out = std::clamp(v_oa, this->v_oa_min_, this->v_oa_max_);

  ESP_LOGI(TAG,
           "[%s] sp=%dppm, cur=%dppm, e_db=%dppm, dt=%.1fs, ib=%.2fmin*ppm, raw=%.2fm3/h, out=%.2fm3/h [%.0f..%.0fm3/h] (kp=%.4f, ti=%.1f, db=%d)",
           PI_CONTROLLER_VERSION, setpoint, current, e_db, dt_s, this->ib_, v_oa, v_oa_out,
           this->v_oa_min_, this->v_oa_max_, this->kp_, this->ti_, this->db_);

  return v_oa_out;
}

}  // namespace auto_co2
}  // namespace tion
}  // namespace dentra
