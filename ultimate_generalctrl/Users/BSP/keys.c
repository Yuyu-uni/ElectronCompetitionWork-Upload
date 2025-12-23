#include "keys.h"

void keys_init(KeyStruct *keys, uint8_t keys_num)
{
    keys->KeyNum = keys_num;
    keys->state = 0x00; // 初始化按键状态为0
    switch (keys_num)
    {
    case 1:
        keys->port = USW1_Core_GPIO_Port;
        keys->pin = USW1_Core_Pin;
        break;
    case 2:
        keys->port = USW2_Core_GPIO_Port;
        keys->pin = USW2_Core_Pin;
        break;
    case 3:
        keys->port = USW3_Driver_GPIO_Port;
        keys->pin = USW3_Driver_Pin;
        break;
    case 4:
        keys->port = USW4_Driver_GPIO_Port;
        keys->pin = USW4_Driver_Pin;
        break;

    default:
        break;
    }
    keys->key_count = 0;                // 初始化按键计数
    keys->currentState = KEY_UNPRESSED; // 初始化当前按键状态为未
    keys->prevState = KEY_UNPRESSED;    // 初始化上一次按键状态为未按下
    keys->S = 0;                        // 初始化状态机状态为0
    keys->thresholdTime = 0;            // 初始化时间阈值为0
    keys->lastReadDuration = 0;         // 初始化上次读取状态的时间间隔为0
}
/**
 * @brief 读取按键状态
 * @param keys: 按键结构体指针
 * @param flag_mask: 标志位掩码，用于指定要读取的按键状态
 * @note 用户在该函数中获取按键状态 建议使用时只调用这一个函数
 * @return 返回1表示指定按键状态被检测到（如按下、释放等），返回0表示未检测到该状态
 */
uint8_t keys_readState(KeyStruct *keys, uint8_t flag_mask)
{
    if (keys->lastReadDuration < KEY_STATE_ACTIVE_TIME)
    {
        keys->lastReadDuration = 0; // 重置读取时间间隔
        // 如果上次读取状态的时间间隔小于可信时间，直接返回当前状态
        if (keys->state & flag_mask)
        {
            if (flag_mask != KEY_HOLD_MASK)
            {
                keys->state &= ~flag_mask; // 清除不是hold的标志位
            }

            return 1;
        }
        return 0; // 返回按键状态
    }
    else
    {
        // 如果上次读取状态的时间间隔大于可信时间，重置状态
        keys->state = 0x00;         // 重置按键状态
        keys->lastReadDuration = 0; // 重置读取时间间隔
        return 0;                   // 返回重置后的状态
    }
}

/**
 * @brief 由tick函数获取按键状态
 * @param keys: 按键结构体指针
 * @return 返回按键状态，0表示未按下，1表示按下 为方便阅读，已在宏定义中改写
 * @note 注意：该函数需要在keys_tick里调用 每KEY_SCAN_COUNT（ms）调用一次 不建议用户在主逻辑中调用
 */
uint8_t keys_getState(KeyStruct *keys)
{
    if (HAL_GPIO_ReadPin(keys->port, keys->pin) == GPIO_PIN_RESET)
    {
        // 按键按下
        return KEY_PRESSED;
    }
    else
    {
        // 按键未按下
        return KEY_UNPRESSED;
    }
}

/*
 * @brief 按键每ms状态扫描函数
 * @note 注意：该函数需要在定时器中断里调用 每1ms调用一次
 */
void keys_tick(KeyStruct *keys)
{

    keys->lastReadDuration++; // 记录读取状态的时间间隔

    if (keys->thresholdTime > 0)
    {
        keys->thresholdTime--; // 每次tick减少1ms
    }

    keys->key_count++;
    // 按键消抖，超过20ms才进行状态检测
    if (keys->key_count >= KEY_SCAN_COUNT)
    {
        keys->key_count = 0;

        keys->prevState = keys->currentState;     // 保存上一次状态
        keys->currentState = keys_getState(keys); // 获取当前按键状态
        // 按键HOLD标志位处理-------------------------------------
        /*
         * Hold标志位会在tick函数中自动置1或置0 这是其余所有按键状态检测的基础
         */
        if (keys->currentState == KEY_PRESSED)
        {
            // Hold标志位置1
            keys->state |= KEY_HOLD_MASK; // 设置Hold标志位
        }
        else
        {
            // Hold标志位置0
            keys->state &= ~KEY_HOLD_MASK; // 清除Hold标志位
        }
        //-----------------------------------------------------

        // 按键DOWN标志位处理------------------------------------
        /*
         * DOWN标志位只在tick函数中置1.这类状态由多处置0
         */
        if (keys->currentState == KEY_PRESSED && keys->prevState == KEY_UNPRESSED)
        {
            // down标志位置1
            keys->state |= KEY_DOWN_MASK; // 设置DOWN标志位
        }
        //-----------------------------------------------------
        // 按键UP标志位处理--------------------------------------
        /*
         * UP标志位只在tick函数中置1.这类状态由多处置0
         */
        if (keys->currentState == KEY_UNPRESSED && keys->prevState == KEY_PRESSED)
        {
            // up标志位置1
            keys->state |= KEY_UP_MASK; // 设置UP标志位
        }
        //-----------------------------------------------------

        // 按键状态机处理----------------------------------------
        switch (keys->S)
        {
        case 0:
            if (keys->currentState == KEY_PRESSED)
            {
                keys->thresholdTime = KEY_HOLD_TIME; // 为长按时间阈值赋初值
                keys->S = 1;                         // 进入按键按下状态
            }

            break;
            // 松开或者长按阈值到了就进下一个状态
        case 1:
            if (keys->currentState == KEY_UNPRESSED)
            {
                keys->thresholdTime = KEY_DOUBLE_TIME; // 为双击时间阈值赋初值
                keys->S = 2;
            }
            else if (keys->thresholdTime == 0 /*长按时间到*/)
            {
                keys->thresholdTime = KEY_REPEAT_TIME;
                keys->state |= KEY_LONG_MASK; // 设置LONG标志位
                // LONG标志位置1
                keys->S = 4;
            }

            break;
        case 2:
            if (keys->currentState == KEY_PRESSED)
            {
                keys->state |= KEY_DOUBLE_MASK; // 设置DOUBLE标志位
                // DOUBLE标志位置1
                keys->S = 3; // 按键已双击
            }
            else if (keys->thresholdTime == 0)
            {
                keys->state |= KEY_SINGLE_MASK; // 设置SINGLE标志位
                // 双击时间到 一次单击完成 SIGNAL位置1
                keys->S = 0;
            }

            break;
        case 3:
            if (keys->currentState == KEY_UNPRESSED)
            {
                // 按键松开，双击完成
                keys->S = 0; // 回到初始状态
            }

            break;
        case 4:
            if (keys->currentState == KEY_UNPRESSED)
            {
                // 按键松开，长按完成
                keys->S = 0; // 回到初始状态
            }
            else if (keys->thresholdTime == 0)
            {
                keys->thresholdTime = KEY_REPEAT_TIME; // 重置重复触发时间
                // 按键持续按下，重复触发
                keys->state |= KEY_REPEAT_MASK; // 设置REPEAT标志位
                keys->S = 4;                    // 保持在长按状态
            }

            break;
        default:
            keys->S = 0; // 出现未知状态，重置到初始状态
            break;
        }
    }
}
