/*  Corresponded with Raspberry PI  */
#include "HC.h"

bool HC::check_robust_arm()
{
    if (control_mode == HC_ROBUST)
    {
        if (motors.armed())
        {
            IS_ARM = 0x01;
        }
        else
        {
            IS_ARM = 0x00;
        }
    }
    else 
        IS_ARM = 0x00;
    if(IS_ARM != 0x00) return true;
    else return false;
}

void HC::communication_rasp()
{
    if (control_mode == HC_ROBUST)
    {
        if (motors.armed())
        {
            cal_ciscrea_angle();
            send_to_rasp();
            receive_from_rasp();
        }
        else
        {
            send_to_rasp();
            init_disarm_ciscrea();
        }
    }
    else
    {
        send_to_rasp();
        init_disarm_ciscrea();
    }    
}

void HC::send_to_rasp()
{
    //	float code_torque = 0.0;
    //	real_angle = 456.12378;
    //	code_torque = real_angle;
    //	float error_angle = target_angle - real_angle;

    //	tran_angle.angleX = error_angle;
    tran_angle.angleX = target_angle - real_angle;

    //	_buffertx[0] = 0x3A;
    //	_bufferrx[1] = 0x3B;
    //	_bufferrx[2] = 0x11;
    //	_bufferrx[3] = 0x12;
    //	_bufferrx[4] = 0x12;
    //	_bufferrx[5] = 0x20;
    //	_bufferrx[6] = 0x7E;
    //	_bufferrx[7] = 0x7F;

    _buffertx[0] = 0x3A;
    _buffertx[1] = 0x3B;
    if (IS_ARM)
    {
        _buffertx[2] = tran_angle.angle_char[0];
        _buffertx[3] = tran_angle.angle_char[1];
        _buffertx[4] = tran_angle.angle_char[2];
        _buffertx[5] = tran_angle.angle_char[3];
    }
    else
    {
        _buffertx[2] = 0x00;
        _buffertx[3] = 0x00;
        _buffertx[4] = 0x00;
        _buffertx[5] = 0x00;
    }

    _buffertx[6] = IS_ARM;
    _buffertx[7] = 0x7E;
    _buffertx[8] = 0x7F;
    //	hal.uartC->printf("error_angle:%f\n",error_angle);

    //	int i = 0;
    //	while(i < 8){
    //		hal.uartD->write(_buffertx[i]);
    //		hal.scheduler->delay(10);
    //		i += 1;
    //	}
    hal.uartD->write(_buffertx[0]);
    hal.uartD->write(_buffertx[1]);
    hal.uartD->write(_buffertx[2]);
    hal.uartD->write(_buffertx[3]);
    hal.uartD->write(_buffertx[4]);
    hal.uartD->write(_buffertx[5]);
    hal.uartD->write(_buffertx[6]);
    hal.uartD->write(_buffertx[7]);
    hal.uartD->write(_buffertx[8]);
    //	for(int j = 0; j < 8; j++){
    //		hal.uartD->printf("%c\n",_buffertx[i]);
    //		hal.scheduler->delay(10);
    //	}
    //	hal.uartD->write(const uint8_t * buffer, size_t size)

    _buffertx[0] = 0x00;
    _buffertx[1] = 0x00;
    _buffertx[2] = 0x00;
    _buffertx[3] = 0x00;
    _buffertx[4] = 0x00;
    _buffertx[5] = 0x00;
    _buffertx[6] = 0x00;
    _buffertx[7] = 0x00;
    _buffertx[8] = 0x00;

    //	hal.uartC->printf("real_angle:%f\n",real_angle);
}



void HC::receive_from_rasp()
{
    int16_t numc;

    numc = hal.uartD->available();

    //	hal.uartD->printf("numc:%d\n",numc);

    int tnum = 0;
    if (numc)
    {
        while (tnum < numc)
        {
            _bufferrx[tnum] = hal.uartD->read();
            //			hal.uartD->printf("receive:%c\n",_buffer[i]);
            if (f_h_flag == 1)
            { //有帧头，判断帧尾，接收消息
                if (f_t1_flag == 1)
                { //有帧头，有帧尾1
                    if (_bufferrx[tnum] == Frame_Tail2)
                    {

                        int i = 0;
                        for (i = 2; i < (tnum - 2); i++)
                        {
                            tran_force.force_char[i - 2] = _bufferrx[i];
                        }
                        torque = tran_force.forceX;
                        //                        for (i = 2; i < (tnum - 1); i++)
                        //                        {
                        //                            hal.uartC->write(_bufferrx[i]);	// 通过串口发送字节
                        //                        }

                        tnum = 0;
                    }
                    else
                    {
                        f_t1_flag = 0;
                        tnum++;
                    }
                }
                else
                { // 有帧头，无帧尾1
                    if (_bufferrx[tnum] == Frame_Tail1)
                    {
                        f_t1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum++;
                    }
                }
            }
            else
            {
                if (f_h1_flag == 1)
                {
                    if (_bufferrx[tnum] == Frame_Header2)
                    {
                        f_h_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        f_h1_flag = 0;
                        tnum = 0;
                    }
                }
                else
                {
                    if (_bufferrx[tnum] == Frame_Header1)
                    {
                        f_h1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum = 0;
                    }
                }
            }
        }
    }

}
void HC::cal_ciscrea_angle()
{
    target_angle = g.cis_heading;

    X1_N_1 = CIS_A[0] * X1_N + CIS_A[1] * X2_N + 0.0;
    X2_N_1 = CIS_A[2] * X1_N + CIS_A[3] * X2_N + CIS_B[1] * torque;
    //	hal.uartC->printf("X1_N:%f\n",X1_N);
    //	hal.uartC->printf("X2_N:%f\n",X2_N);
    //	hal.uartC->printf("torque:%f\n",torque);
    real_angle = CIS_C[0] * X1_N;
    X1_N = X1_N_1;
    X2_N = X2_N_1;
}


//CISCREA para init
void HC::init_mod_ciscrea()
{
    //	X1_N = ahrs.yaw_sensor;
    X1_N = 0.0;
    X2_N = 0.0;
    X1_N_1 = 0.0;
    X2_N_1 = 0.0;
    torque = 0.0;
    target_angle = 5.0;
    f_h1_flag = 0; // 接收到帧头的第一个字节标志位
    f_h_flag = 0;  // 接收到帧头标志位
    f_t1_flag = 0; // 接收到帧尾的第一个字节标志位
    real_angle = 0.0;
    tran_angle.angleX = 0.0;
    tran_force.forceX = 0.0;
    IS_ARM = 0x00;
}

void HC::init_disarm_ciscrea()
{
    X1_N = 0.0;
    X2_N = 0.0;
    X1_N_1 = 0.0;
    X2_N_1 = 0.0;
    torque = 0.0;
    tran_angle.angleX = 0.0;
    tran_force.forceX = 0.0;
}

