#include "protocol.h"
#include "flash_storage.h"
#include "usbd_hid_if.h"
#include "stm32f4xx_hal.h"
#include <string.h>

volatile PendingCall_t g_pending_call = PENDING_CALL_NONE;
volatile bool          g_pending_save = false;

static void _noop_set_state(uint8_t s) { (void)s; }
static void _noop_set_pos  (float  p) { (void)p; }
static void _noop_void     (void)     { }

static ProtocolCallbacks_t s_cb = {
    .set_axis_state = _noop_set_state,
    .set_input_pos  = _noop_set_pos,
    .clear_errors   = _noop_void,
    .enter_dfu      = _noop_void,
    .apply_config   = _noop_void,
};

void protocol_init(const ProtocolCallbacks_t *callbacks)
{
    if (callbacks) {
        if (callbacks->set_axis_state) s_cb.set_axis_state = callbacks->set_axis_state;
        if (callbacks->set_input_pos)  s_cb.set_input_pos  = callbacks->set_input_pos;
        if (callbacks->clear_errors)   s_cb.clear_errors   = callbacks->clear_errors;
        if (callbacks->enter_dfu)      s_cb.enter_dfu      = callbacks->enter_dfu;
        if (callbacks->apply_config)   s_cb.apply_config   = callbacks->apply_config;
    }
}

void protocol_dispatch(const HID_Command_t *cmd)
{
    if (!cmd) return;

    const uint8_t  cid = cmd->cmd_id;
    const uint16_t pid = cmd->param_id;
    const float    val = cmd->value;

    switch (cid) {

        case HID_CMD_SET_AXIS_STATE:
            s_cb.set_axis_state((uint8_t)val);
            break;

        case HID_CMD_SET_INPUT_POS:
            s_cb.set_input_pos(val);
            break;

        case HID_CMD_SET_CURRENT_LIM:
            config_set(CFG_MOTOR_CURRENT_LIM, val);
            break;

        case HID_CMD_CLEAR_ERRORS:
            s_cb.clear_errors();
            break;

        case HID_CMD_ENTER_DFU:
            s_cb.enter_dfu();
            break;

        case HID_CMD_SET_CONFIG:
            config_set(pid, val);
            break;

        case HID_CMD_GET_CONFIG: {
            float out = 0.0f;

            if (config_get(pid, &out)) {
                printf("GET OK ID=%d VALUE=%f\n", pid, out);

                protocol_send_config_response(pid, out);
            } else {
                printf("GET FAIL ID=%d\n", pid);
            }

            break;
        }

        case HID_CMD_CALL: {
            uint16_t call_id = pid ? pid : (uint16_t)val;
            switch (call_id) {
                case CALL_MOTOR_CALIBRATION:
                    g_pending_call = PENDING_CALL_MOTOR_CAL;       break;
                case CALL_ENCODER_CALIBRATION:
                    g_pending_call = PENDING_CALL_ENCODER_CAL;     break;
                case CALL_ENCODER_INDEX_SEARCH:
                    g_pending_call = PENDING_CALL_ENCODER_INDEX;   break;
                case CALL_ENTER_CLOSED_LOOP:
                    g_pending_call = PENDING_CALL_CLOSED_LOOP;     break;
                case CALL_CLEAR_ERRORS:
                    s_cb.clear_errors();                            break;
                case CALL_SET_IDLE:
                    g_pending_call = PENDING_CALL_SET_IDLE;        break;
                case CALL_APPLY_CONFIG:
                    g_pending_call = PENDING_CALL_APPLY_CONFIG;    break;
                default: break;
            }
            break;
        }

        case HID_CMD_SAVE_CONFIG:
            g_pending_save = true;
            break;

        case HID_CMD_REBOOT:
            HAL_Delay(10);
            NVIC_SystemReset();
            break;

        default:
            break;
    }
}

void protocol_process_pending(void)
{
    PendingCall_t call = g_pending_call;
    if (call != PENDING_CALL_NONE) {
        g_pending_call = PENDING_CALL_NONE;

        switch (call) {
            case PENDING_CALL_MOTOR_CAL:
                s_cb.set_axis_state(4 /* AXIS_STATE_MOTOR_CALIBRATION */);
                break;
            case PENDING_CALL_ENCODER_CAL:
                s_cb.set_axis_state(7 /* AXIS_STATE_ENCODER_OFFSET_CALIBRATION */);
                break;
            case PENDING_CALL_ENCODER_INDEX:
                s_cb.set_axis_state(6 /* AXIS_STATE_ENCODER_INDEX_SEARCH */);
                break;
            case PENDING_CALL_CLOSED_LOOP:
                s_cb.set_axis_state(8 /* AXIS_STATE_CLOSED_LOOP_CONTROL */);
                break;
            case PENDING_CALL_CLEAR_ERRORS:
                s_cb.clear_errors();
                break;
            case PENDING_CALL_SET_IDLE:
                s_cb.set_axis_state(1 /* AXIS_STATE_IDLE */);
                break;
            case PENDING_CALL_APPLY_CONFIG:
                s_cb.apply_config();
                break;
            default:
                break;
        }
    }

    if (g_pending_save) {
        g_pending_save = false;
        flash_save_config();
    }
}

bool protocol_send_config_response(uint16_t param_id, float value)
{
    HID_ConfigResponse_t resp;
    resp.report_id = HID_REPORT_ID_CONFIG_RESP;
    resp.param_id  = param_id;
    resp.value     = value;
    return (HID_ODrive_SendConfigResponse(&resp) == USBD_OK);
}
