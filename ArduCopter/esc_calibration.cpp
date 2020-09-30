#include "Copter.h"

/*****************************************************************************
* Functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// 检查是否应进入esc校准模式
// check if we should enter esc calibration mode
void Copter::esc_calibration_startup_check()
{
    if (motors->get_pwm_type() == AP_Motors::PWM_TYPE_BRUSHED) {
        // ESC cal对有刷电机无效
        // ESC cal not valid for brushed motors
        return;
    }

#if FRAME_CONFIG != HELI_FRAME
    // 第一个无线电输入最多延迟2秒
    // delay up to 2 second for first radio input
    uint8_t i = 0;
    while ((i++ < 100) && (last_radio_update_ms == 0)) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // 如果pre-arm rc检查失败，则立即退出
    // exit immediately if pre-arm rc checks fail
    if (!arming.rc_calibration_checks(true)) {
        // 下次清除esc标志
        // clear esc flag for next time
        if ((g.esc_calibrate != ESCCalibrationModes::ESCCAL_NONE) && (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED)) {
            g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
        }
        return;
    }

    // 检查ESC参数
    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCalibrationModes::ESCCAL_NONE:
            // check if throttle is high--检查油门是否高
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                //下次重启时，我们将进入esc_calibrate模式
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs--向gcs发送消息
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
                // turn on esc calibration notification--打开esc校准通知
                AP_Notify::flags.esc_calibration = true;
                // block until we restart--阻止，直到我们重新启动
                while(1) { hal.scheduler->delay(5); }
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high--检查油门是否高
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // pass through pilot throttle to escs--通过先导油门到达escs
                esc_calibration_passthrough();
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_ALWAYS:
            // pass through pilot throttle to escs--通过先导油门到达escs
            esc_calibration_passthrough();
            break;
        case ESCCalibrationModes::ESCCAL_AUTO:
            // perform automatic ESC calibration--执行自动ESC校准
            esc_calibration_auto();
            break;
        case ESCCalibrationModes::ESCCAL_DISABLED:
        default:
            // do nothing
            break;
    }

    // clear esc flag for next time
    if (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED) {
        g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_passthrough-通过先导油门到达escs
// esc_calibration_passthrough - pass through pilot throttle to escs
void Copter::esc_calibration_passthrough()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

    esc_calibration_setup();

    while(1) {
        // flash LEDs--闪光灯LED
        esc_calibration_notify();

        // read pilot input--读取飞行员输入
        read_radio();

        // 我们以很高的速度运行，以使单发ESC开心。普通ESC仅在RC_SPEED处看到脉冲
        // we run at high rate to make oneshot ESCs happy. Normal ESCs
        // will only see pulses at the RC_SPEED
        hal.scheduler->delay(3);

        // pass through to motors--通过电机
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() / 1000.0f);
        SRV_Channels::push();
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_auto-使用计时器自动校准ESC，无需先导输入
// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    esc_calibration_setup();

    // raise throttle to maximum
    SRV_Channels::cork();
    motors->set_throttle_passthrough_for_esc_calibration(1.0f);
    SRV_Channels::push();

    // delay for 5 seconds while outputting pulses
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // block until we restart
    while(1) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }
#endif // FRAME_CONFIG != HELI_FRAME
}

//  闪烁的LED通知用户ESC校准正在进行中
// flash LEDs to notify the user that ESC calibration is happening
void Copter::esc_calibration_notify()
{
    AP_Notify::flags.esc_calibration = true;
    uint32_t now = AP_HAL::millis();
    if (now - esc_calibration_notify_update_ms > 20) {
        esc_calibration_notify_update_ms = now;
        notify.update();
    }
}

void Copter::esc_calibration_setup()
{
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    if (motors->get_pwm_type() >= AP_Motors::PWM_TYPE_ONESHOT) {
        // run at full speed for oneshot ESCs (actually done on push)
        motors->set_update_rate(g.rc_speed);
    } else {
        // reduce update rate to motors to 50Hz
        motors->set_update_rate(50);
    }

    // disable safety if requested--根据要求禁用安全性
    BoardConfig.init_safety();

    // wait for safety switch to be pressed--等待按下安全开关
    uint32_t tstart = 0;
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        const uint32_t tnow = AP_HAL::millis();
        if (tnow - tstart >= 5000) {
            gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            tstart = tnow;
        }
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // arm and enable motors--手臂启动马达
    motors->armed(true);
    SRV_Channels::enable_by_mask(motors->get_motor_mask());
    hal.util->set_soft_armed(true);
}
