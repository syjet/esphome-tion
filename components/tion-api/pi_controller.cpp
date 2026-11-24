#include <algorithm>
#include "log.h"
#include "pi_controller.h"

namespace dentra {
namespace tion {
namespace auto_co2 {

static const char *const TAG = "pi_controller";

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
  // 7: error from setpoint [ppm]
  const auto e = setpoint - current;

  // 8: error from setpoint, including dead band [ppm]
  const auto e_db =  //
      /**/ current < setpoint - this->db_ ? e - this->db_ :
      /**/ current > setpoint + this->db_ ? e + this->db_
                                          : 0;

  // 9: integral error [min.ppm-CO2]
  const float dt_s = this->dt_s_();
  const float i = this->ib_ + (dt_s / 60.0f) * e_db;

  // 10: candidate outdoor airflow rate [m3/h]
  const float v_oa_c = -this->kp_ * (e_db + (i / this->ti_));

  // Базовый интеграл, соответствующий минимальному притоку без компенсации отрицательной ошибки
  const float base_min_ib = -this->ti_ * (this->v_oa_min_ / this->kp_);

  // 11: integral error with anti-integral windup [min.ppm-CO2]
  if (v_oa_c < this->v_oa_min_) {
    const float windup_min_ib = -this->ti_ * (e_db + (this->v_oa_min_ / this->kp_));
    // Не позволяем интегралу задираться выше базового минимума при чистом воздухе (e_db > 0)
    this->ib_ = std::max(windup_min_ib, base_min_ib);
  } else if (v_oa_c > this->v_oa_max_) {
    this->ib_ = -this->ti_ * (e_db + (this->v_oa_max_ / this->kp_));
  } else {
    this->ib_ = i;
  }

  // 12: outdoor airflow rate [m3/h]
  const float v_oa = -this->kp_ * (e_db + (this->ib_ / this->ti_));
  const float v_oa_out = std::clamp(v_oa, this->v_oa_min_, this->v_oa_max_);

  ESP_LOGI(TAG,
           "sp=%dppm, cur=%dppm, dt=%.0fs, ib=%.2fmin*ppm, raw=%.2fm3/h, out=%.2fm3/h [%.0f..%.0fm3/h] (kp=%.4f, ti=%.1fmin, db=%dppm)",
           setpoint, current, dt_s, this->ib_, v_oa, v_oa_out, this->v_oa_min_, this->v_oa_max_, this->kp_, this->ti_, this->db_);

  return v_oa_out;
}

}  // namespace auto_co2
}  // namespace tion
}  // namespace dentra
