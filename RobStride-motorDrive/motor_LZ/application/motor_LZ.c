#include "motor_LZ.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static Motor_LZ_Init motor_LZ_id[motor_LZ_N]={
//				{1,MOTOR_LZ_05},
				{2,MOTOR_LZ_02}
};

static uint8_t motor_LZ_id_seat[8];
static motor_LZ motor_LZ_send_recv[motor_LZ_N];

static CAN_TxHeaderTypeDef  motor_LZ_tx;

static fp32 motor_LZ_max_min(fp32 proto,fp32 max,fp32 min){
	if(proto>max)return max;
	else if(proto<min) return min;
	return proto;
} 

/**
  * @brief          ��������ʼ��
  * @param[in]      none
  * @retval         none
  */
void motor_LZ_init(){
	can_filter_init();
	for(uint8_t i=0;i<motor_LZ_N;++i){
		motor_LZ_send_recv[i].id = motor_LZ_id[i].id;
		motor_LZ_id_seat[motor_LZ_id[i].id]=i;
		motor_LZ_send_recv[i].DataHanding.exld.id=motor_LZ_id[i].id;
		motor_LZ_send_recv[i].DataHanding.exld.res = 0;
		motor_LZ_send_recv[i].enable = 0;
		motor_LZ_send_recv[i].model = motor_LZ_id[i].model;
	}
	for(uint8_t i=0;i<motor_LZ_N;++i){
		while(motor_LZ_send_recv[i].Data_recv.Temp==0){
			motor_LZ_enable(motor_LZ_id[i].id);
			HAL_Delay(1);
		}
		motor_LZ_active_recv(motor_LZ_id[i].id,01);
		motor_LZ_send_recv[i].enable = 1;
		motor_LZ_send_recv[i].Data_send->Angle = motor_LZ_send_recv[i].Data_recv.Angle;
	}

}

/**
  * @brief          �������ⲿ���Ʋ�����ȡ
  * @param[in]      id ���id
  * @param[in]    	*Data_send ����ָ��
  * @retval         none
  */
void motor_LZ_send_init(uint8_t id,user_send_Lz  *Data_send){
	Data_send->Angle = motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_recv.Angle;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send = Data_send;
}

/**
  * @brief          �������������ݻ�ȡ
  * @param[in]      id ���id
  * @retval         user_recv_Lz ����ָ��
  */
const user_recv_Lz *motor_LZ_recv_return(uint8_t id){
	return &motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_recv;
}

/**
  * @brief          ���������ݷ���
  * @param[in]      id ���id
  * @retval         none
  */
static void motor_LZ_Handle_send(uint8_t id){
	uint32_t motor_LZ_send_box;
	motor_LZ_tx.ExtId = motor_LZ_send_recv[id].DataHanding.exld.mode<<24
						|motor_LZ_send_recv[id].DataHanding.exld.data<<8|motor_LZ_send_recv[0].DataHanding.exld.id;
	motor_LZ_tx.IDE = CAN_ID_EXT ;
	motor_LZ_tx.RTR = CAN_RTR_DATA ;
	motor_LZ_tx.DLC = motor_LZ_send_recv[id].DataHanding.len;
	HAL_CAN_AddTxMessage(&LZ_CAN, &motor_LZ_tx, motor_LZ_send_recv[id].DataHanding.Data, &motor_LZ_send_box);

}

/**
  * @brief          �������������ݴ���
  * @param[in]      id ���id
  * @param[in]		������ݴ�������
  * @param[in]		model����ͺ�
  * @retval         none
  */
static void motor_LZ_send_handle(uint8_t id,uint16_t *motor,uint8_t model){
	switch(model){
		case MOTOR_LZ_02:
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle = 
					motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle,12.57f,-12.57f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W,44.00f,-44.00f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->T =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->T,17.0f,-17.0f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd,5.0f,0.0f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp,500.0f,0.0f);
			motor[0]=(uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle+12.57f)*65535/25.14f);
			motor[1] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W+44.00f)*65535/88.0f);
			motor[2] = (uint16_t)((motor_LZ_send_recv[id].Data_send->T+17.0f)*65535/34.0f);
			motor[3] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp*65535/500.0f));
			motor[4] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd*65535/5.00f));
		break;
		case MOTOR_LZ_05 :
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle = 
					motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle,12.57f,-12.57f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W,50.00f,-50.00f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->T =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->T,5.5f,-5.5f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd,5.0f,0.0f);
			motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp =
								motor_LZ_max_min(motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp,500.0f,0.0f);
			motor[0]=(uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Angle+12.57f)*65535/25.14f);
			motor[1] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->W+50.0f)*65535/100.0f);
			motor[2] = (uint16_t)((motor_LZ_send_recv[id].Data_send->T+5.5f)*65535/11.0f);
			motor[3] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kp*65535/500.0f));
			motor[4] = (uint16_t)((motor_LZ_send_recv[motor_LZ_id_seat[id]].Data_send->Kd*65535/5.0f));
		break;
		default:
		{
			break;
		}
	}
	
}

/**
  * @brief          �������������ݴ���
  * @param[in]      id ���id
  * @retval         none
  */
void motor_LZ_send(uint8_t id){
	uint16_t motor_LZ_send[5];
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = CANCOM_MOTOR_CTRL;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	
	motor_LZ_send_handle(id,motor_LZ_send,motor_LZ_send_recv[motor_LZ_id_seat[id]].model);
	
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = motor_LZ_send[2];
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[0]=motor_LZ_send[0]>>8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[1]=motor_LZ_send[0];
	
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[2]=motor_LZ_send[1]>>8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[3]=motor_LZ_send[1];
	
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[4]=motor_LZ_send[3]>>8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[5]=motor_LZ_send[3];	
	
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[6]=motor_LZ_send[4]>>8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[7]=motor_LZ_send[4];
	
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);

}

/**
  * @brief          ������ʹ��
  * @param[in]      id ���id
  * @retval         none
  */
void motor_LZ_enable(uint8_t id){
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = CANCOM_MOTOR_IN;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = motor_LZ_user_id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	for(uint8_t i=0;i<8;++i){
		motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[i]=0;
	}
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);

}

/**
  * @brief          ������ʧ��
  * @param[in]      id ���id
  * @retval         none
  */
void motor_LZ_lose(uint8_t id){
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = CANCOM_MOTOR_RESET;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = motor_LZ_user_id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	for(uint8_t i=0;i<8;++i){
		motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[i]=0;
	}
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);
	motor_LZ_send_recv[motor_LZ_id_seat[id]].enable = 0;
}

/**
  * @brief          �����������ϱ�֡
  * @param[in]      id ���id
  * @param[in]		F_CMDģʽ���� 00�� 01����
  * @retval         none
  */
void motor_LZ_active_recv(uint8_t id,uint8_t F_CMD){
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = 0x18;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = motor_LZ_user_id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	for(uint8_t i=0;i<6;++i){
		motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[i]=i+1;
	}
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[6]=(F_CMD==1);
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);

}
/**
  * @brief          ���������û�е��� ��ѵ�ǰ���λ����Ϊ��е��λ�� ����ʧ�ܵ��, ��ʹ�ܵ��
  * @param[in]      id ���id
  * @retval         none
  */
void motor_LZ_zero(uint8_t id){
	motor_LZ_lose(id);
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = CANCOM_MOTOR_ZERO;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = motor_LZ_user_id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[0]=1;
	
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);
	motor_LZ_enable(id);
}
/**
  * @brief          ����������CAN_ID  ʧ�ܵ��
  * @param[in]      id ���id
  * @param[in] 		�޸ĺ�Ԥ�裩CANID
  * @retval         none
  */
void motor_LZ_set_CAN_ID(uint8_t id,uint8_t set_id){
	motor_LZ_lose(id);
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.id = id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.mode = CANCOM_MOTOR_ZERO;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.exld.data = set_id;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.len = 8;
	motor_LZ_send_recv[motor_LZ_id_seat[id]].DataHanding.Data[0]=1;
	
	motor_LZ_Handle_send(motor_LZ_id_seat[id]);
	motor_LZ_enable(id);
}

/**
  * @brief          �������������ݻ�ȡ
  * @param[in]      id ���id
  * @param[in]		Data ExtId���ݴ��� 
  * @param[in]		data����
  * @param[in]		model����ͺ�
  * @retval         none
  */
static void motor_LC_recv_data(uint8_t id, uint32_t Data , uint8_t *data,uint8_t model){
	
	motor_LZ_send_recv[id].Data_recv.Angle = (fp32)(data[0]<<8|data[1])*(fp32)12.57f/32768-12.57f;
	motor_LZ_send_recv[id].Data_recv.Temp  = (fp32)(data[6]<<8|data[7])/10.0f;
	motor_LZ_send_recv[id].Data_recv.MError= Data>>16&0x3F;
	motor_LZ_send_recv[id].Data_recv.mode  = Data>>22&0x03;
	switch(model){
		case MOTOR_LZ_02:
			motor_LZ_send_recv[id].Data_recv.W     = (fp32)(data[2]<<8|data[3])*(fp32)11.0f/8192-44.0f;
			motor_LZ_send_recv[id].Data_recv.T     = (fp32)(data[4]<<8|data[5])*(fp32)17.0f/32768-17.0f;
		break;
		case MOTOR_LZ_05 :
			motor_LZ_send_recv[id].Data_recv.W     = (fp32)(data[2]<<8|data[3])*(fp32)12.5f/8192-50.0f;
			motor_LZ_send_recv[id].Data_recv.T     = (fp32)(data[4]<<8|data[5])*(fp32)5.5f/32768-5.5f;
		break;
		default:
		{
			break;
		}
	
	}

}
 


/**
  * @brief          can���ջص�����
  * @param[in]      hcan hcanָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	if(hcan == &LZ_CAN){
		
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		uint8_t id = rx_header.ExtId>>8;
		uint8_t mode = (rx_header.ExtId>>24)&0x1F;
		switch(mode){
			case 0x18:
			case 0x2 :
				motor_LC_recv_data(motor_LZ_id_seat[id],rx_header.ExtId,rx_data,motor_LZ_send_recv[motor_LZ_id_seat[id]].model);
			break;
			default:
			{
				break;
			}
		}
	}
	
}
