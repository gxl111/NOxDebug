//////////////////////
//接受任务和传感器任务，MODBUS从机任务定义在这
//////////////////////
#include "NOx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "modbus_slave.h"
#include "modbus_host.h"
#include "oled.h"
#include "sdcard.h"

//转换公式参数 ，第一段0点到标定点1
Parameter NOx_parameter= {DEFAULTNOX_A,DEFAULTNOX_B};
Parameter O2_parameter= {DEFAULTO2_A,DEFAULTO2_B};
//第二段参数
Parameter NOx_parameter1= {DEFAULTNOX_A,DEFAULTNOX_B};
Parameter O2_parameter1= {DEFAULTO2_A,DEFAULTO2_B};

//NOx和O2校准点位
uint16_t NOx_x[NOX_CALIBRATION_NUM];
float NOx_y[NOX_CALIBRATION_NUM]={NOX_Y0,NOX_Y1,NOX_Y2};
uint16_t O2_x[O2_CALIBRATION_NUM];
float O2_y[O2_CALIBRATION_NUM]={O2_Y0,O2_Y1,O2_Y2};

static TimerHandle_t TimerBlowback;

//发送加热指令canID 实际在初始化函数中使用
uint32_t canId = 0x18FEDF55;
//发送加热指令can数据
uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55};

//接收消息保存在此
J1939_MESSAGE TxMessage;
//发送消息保存在此
J1939_MESSAGE RxMessage;

//传感器未经转换数据
uint16_t NOX_raw;
uint16_t O2_raw;

//NOx 单位ppm，O2单位%
float NOx;
float O2;

//报警高低值
float NOx_High =2000;
float O2_Low =15;

//单位s
uint32_t blowtime=60;
//单位s
uint32_t blowspan=3600;
static uint8_t blow_flag=0;

//写寄存器互斥量
SemaphoreHandle_t g_hVarMutex = NULL;

/* 可配置的平均点数 */
//#define NOX_AVG_N   8
//#define O2_AVG_N    8

//static uint16_t nox_raw_buf[NOX_AVG_N] = {0};
//static uint16_t o2_raw_buf[O2_AVG_N]   = {0};
//static uint8_t  nox_idx = 0;
//static uint8_t  o2_idx  = 0;
//uint8_t avg_flag=0;
/* */


static void Calibration_Init(uint16_t x[],float y[],Parameter p, Parameter p1);
static uint8_t Calculate_slope_intercept(uint16_t x[],float y[], Parameter* p);
static void Blowback(void);
static void Alarm(void);
static void change_to_electricity(uint8_t * buf);

void Register_Init(void);
void AfterFlash_Init(void);
void BLOW_CONTROL(uint8_t state);
/*
*********************************************************************************************************
*	函 数 名: NOx_Set_DefaultParameter
*	功能说明: 设置传感器参数为默认值。
*	形    参: Parameter *p：传感器参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
void NOx_Set_DefaultParameter(Parameter *noxp) {
    noxp->a=DEFAULTNOX_A;
    noxp->b=DEFAULTNOX_B;

}
void O2_Set_DefaultParameter(Parameter *o2p) {

    o2p->a=DEFAULTO2_A;
    o2p->b=DEFAULTO2_B;
}

/*
*********************************************************************************************************
*	函 数 名: Calibration
*	功能说明: 远程标定函数。 3点标定8，9为一组NOx，10，11为一组O2，8，10默认为0xffff,发送0x0001进行标定,标定失败返回0x5555，标定成功0xf000  恢复默认值0xffff
*            标定步骤：通标气，待读数稳定，对标定点y值进行确认，并且确认9或11对应标定点 对P08或P10  发送0x0001进行标定
*	形    参: Parameter *noxp,Parameter *o2p：传感器参数指针
*	返 回 值: 无
*********************************************************************************************************
*/
void Calibration_NOx(Parameter *p,Parameter *p1) {
    
		
    //标定主要是改y对应的x值
    
    //此处可以加上控制标气阀通断并且关烟气阀
    
    
    //更新手动修改后的值(可以手动修改参数)
		Var_Read_ParamSection1(&NOx_parameter.a,&NOx_parameter.b,&NOx_parameter1.a,&NOx_parameter1.b);
		
    uint16_t calibration=Var_Read_P08();
    if(calibration==1) {

        switch(g_tVar.P09){
					 // 零点标定
           case 0:
               //更新标定点x的值
               NOx_x[0]= NOX_raw;
               Calculate_slope_intercept(NOx_x,NOx_y,p);

               break;
					 // 第一点标定
           case 1:
							 NOx_y[1]=Var_Read_P18();
               NOx_x[1]= NOX_raw;
					 
               Calculate_slope_intercept(NOx_x,NOx_y,p);
               Calculate_slope_intercept(NOx_x+1,NOx_y+1,p1);
               break;
					 // 第二点标定
           case 2:
							 NOx_y[2]=Var_Read_P20();
               NOx_x[2]= NOX_raw;
					 
               Calculate_slope_intercept(NOx_x+1,NOx_y+1,p1);               
               break;
           default:
               Var_Write_P08(0x0005); 
               return;
        }

        //标定成功
        Var_Write_P08(0x000f);
        
        // 写入flash
				Var_Update_ParamSection1(NOx_parameter.a,NOx_parameter.b,NOx_parameter1.a,NOx_parameter1.b);
				
        InternalFlash_Write();
        
    }
    else  if(calibration==0x02){
        //参数恢复默认
        NOx_Set_DefaultParameter(&NOx_parameter);
        NOx_Set_DefaultParameter(&NOx_parameter1);
        Var_Write_P08(0x0010);
        
        Var_Update_ParamSection1(NOx_parameter.a,NOx_parameter.b,NOx_parameter1.a,NOx_parameter1.b);
        //回复默认后xy也要对应
        Calibration_Init(NOx_x,NOx_y,NOx_parameter,NOx_parameter1);
    
        InternalFlash_Write();
        
    }
        
    return;
}

void Calibration_O2(Parameter *p,Parameter *p1) {
    
     
    //此处可以加上控制标气阀通断并且关烟气阀
    
    
    //更新手动修改后的值	
		Var_Read_ParamSection2(&O2_parameter.a,&O2_parameter.b,&O2_parameter1.a,&O2_parameter1.b);
	
    uint16_t calibration=Var_Read_P10();
    if(calibration==1) {
        switch(g_tVar.P11){
           case 0:
               //更新标定点x的值
               O2_x[0]= O2_raw;
               Calculate_slope_intercept(O2_x,O2_y,p);
               break;
           case 1:
						   O2_y[1]=g_tVar.P19;
               O2_x[1]= O2_raw;
               Calculate_slope_intercept(O2_x,O2_y,p);
               Calculate_slope_intercept(O2_x+1,O2_y+1,p1);
               break;
           case 2:
						   O2_y[2]=g_tVar.P21;
               O2_x[2]= O2_raw;
               Calculate_slope_intercept(O2_x+1,O2_y+1,p1);               
               break;
           default:
               Var_Write_P10(0x0005); 
               return;
        }

        //标定成功
        Var_Write_P10(0x000f);
        
        //写入flash
				Var_Update_ParamSection2(O2_parameter.a,O2_parameter.b,O2_parameter1.a,O2_parameter1.b);        
        
				InternalFlash_Write();
    }
    else  if(calibration==0x02){
        //参数恢复默认
        O2_Set_DefaultParameter(&O2_parameter);
        O2_Set_DefaultParameter(&O2_parameter1);
        Var_Write_P10(0x0010);
 
				Var_Update_ParamSection2(O2_parameter.a,O2_parameter.b,O2_parameter1.a,O2_parameter1.b);             
        
        Calibration_Init(O2_x,O2_y,O2_parameter,O2_parameter1);
        
			  InternalFlash_Write();
    }

    return;
}

/*
*********************************************************************************************************
*	函 数 名: SensorRegUpdata
*	功能说明: 传感器寄存器更新,在数据处理后进行更新
*	形    参: VAR_T *var：寄存器值指针
*	返 回 值: 无
*********************************************************************************************************
*/
void SensorRegUpdata(VAR_T *var) {


}

/*
*********************************************************************************************************
*	函 数 名: TxMsg_Init
*	功能说明: 发送加热数据初始化
*	形    参: J1939_MESSAGE *TxMsgPtr：1939消息指针
*	返 回 值: 无
*********************************************************************************************************
*/

void TxMsg_Init(J1939_MESSAGE *TxMsgPtr) {
    TxMsgPtr->Mxe.Priority=0x06;
    TxMsgPtr->Mxe.PDUFormat=0xFE;
    TxMsgPtr->Mxe.SourceAddress=J1939_Address;
    TxMsgPtr->Mxe.DataLength=8;
    TxMsgPtr->Mxe.PDUSpecific=0xDF;
    TxMsgPtr->Mxe.DataPage=0;
    TxMsgPtr->Mxe.Res=0;
    TxMsgPtr->Mxe.RTR=0;
    for (int i = 0; i < 8; i++) {
        TxMsgPtr->Mxe.Data[i] = data[i];
    }
}

void hexArrayToString(const j1939_uint8_t *array, size_t length, char *result) {
    result[0] = '\0';
    for (size_t i = 0; i < length; i++) {
        char temp[3];
        snprintf(temp, sizeof(temp), "%02X", array[i]);
        strcat(result, temp);
    }
}

void Updata_VarT(void) {

}



void NOx_Handle(J1939_MESSAGE *RxMsgPtr) {
    uint16_t NOx_State=0x00;
    uint8_t buffer[20];
    NOx_State=1<<8|NOx_State;
//    LCD_ShowString(30,70,300,24,24,(uint8_t*)"RX Succeed    ");

    //解析数据

    NOX_raw = (RxMsgPtr->Mxe.Data[1] << 8) | RxMsgPtr->Mxe.Data[0];
    //分段
    if(NOX_raw<=NOx_x[1] ){
        NOx = (NOx_parameter.a * NOX_raw + NOx_parameter.b);    
    }else{
        NOx = (NOx_parameter1.a * NOX_raw + NOx_parameter1.b);  
    }

    //分段
    O2_raw = (RxMsgPtr->Mxe.Data[3] << 8) | RxMsgPtr->Mxe.Data[2];
    if(O2_raw<=O2_x[1] ){
        O2 = (O2_parameter.a * O2_raw + O2_parameter.b);  
    }else{
        O2 = (O2_parameter1.a * O2_raw + O2_parameter1.b);
    }
    
        /* ---------- NOx 原始滑动平均 ---------- */
//    nox_raw_buf[nox_idx] = NOX_raw;
//    if (++nox_idx >= NOX_AVG_N) {
//        nox_idx = 0;
//        avg_flag=1;
//    }
//    
//    uint32_t nox_sum = 0;
//    for (uint8_t i = 0; i < NOX_AVG_N; i++) nox_sum += nox_raw_buf[i];
//    uint16_t NOX_raw_filt = nox_sum / NOX_AVG_N;

//    /* ---------- O2 原始滑动平均 ---------- */
//    o2_raw_buf[o2_idx] = O2_raw;
//    if (++o2_idx >= O2_AVG_N) o2_idx = 0;
//    
//    uint32_t o2_sum = 0;
//    for (uint8_t i = 0; i < O2_AVG_N; i++) o2_sum += o2_raw_buf[i];
//    uint16_t O2_raw_filt = o2_sum / O2_AVG_N;
//    
//    if(!avg_flag){
//        return;
//    }
//    
//        /* 分段线性标定（NOx） */
//    if (NOX_raw_filt <= NOx_x[1]) {
//        NOx = NOx_parameter.a * NOX_raw_filt + NOx_parameter.b;
//    } else {
//        NOx = NOx_parameter1.a * NOX_raw_filt + NOx_parameter1.b;
//    }

//    /* 分段线性标定（O2） */
//    if (O2_raw_filt <= O2_x[1]) {
//        O2 = O2_parameter.a * O2_raw_filt + O2_parameter.b;
//    } else {
//        O2 = O2_parameter1.a * O2_raw_filt + O2_parameter1.b;
//    }
    
    

    uint8_t statusByte = RxMsgPtr->Mxe.Data[4];

    uint8_t voltageInRange = (statusByte & 0x03);

    uint8_t sensorAtTemp = (statusByte >> 2) & 0x03;

    uint8_t NOxStable = (statusByte >> 4) & 0x03;

    uint8_t O2Stable = (statusByte >> 6) & 0x03;

    uint8_t heaterByte = RxMsgPtr->Mxe.Data[5];

    uint8_t heaterControl = (heaterByte >> 5) & 0x03;
    
    uint8_t errorheater= heaterByte&0x1f;
    
    uint8_t errorNOx = RxMsgPtr->Mxe.Data[6]&0x1f;
    uint8_t errorO2 = RxMsgPtr->Mxe.Data[7]&0x1f;

    //uint8_t heaterFMI = heaterByte & 0x1F;
    
    //NOx信号
    if(NOxStable == 0x01) {
        strcpy((char *)buffer, "NOx: stable ");
        NOx_State=1<<7|NOx_State;

    } else if(NOxStable == 0x00) {
        strcpy((char *)buffer, "NOx: invalid  ");

    } else if(NOxStable == 0x10) {
        strcpy((char *)buffer, "NOx: no use  ");
    } else {
        strcpy((char *)buffer, "NOx: default  ");
    }
//    LCD_ShowString(30,100,210,24,24,buffer);
    snprintf((char *)buffer, sizeof(buffer), "NOx: %5.2f ppm    ", NOx);
    OLED_PrintASCIIString(0, 10, (char *)buffer, &afont16x8, OLED_COLOR_NORMAL);

    //O2信号
    if(O2Stable == 0x01) {
        strcpy((char *)buffer, "O2 : stable ");
        NOx_State=1<<6|NOx_State;
    } else if(O2Stable == 0x00) {
        strcpy((char *)buffer, "O2 : invalid  ");
    } else if(O2Stable == 0x10) {
        strcpy((char *)buffer, "O2 : no use  ");

    } else {
        strcpy((char *)buffer, "O2: default  ");
    }

    snprintf((char *)buffer, sizeof(buffer), "O2 : %5.2f %         % ", O2);
    //LCD_ShowString(30,190,210,24,24,buffer);
    OLED_PrintASCIIString(0, 30, (char *)buffer, &afont16x8, OLED_COLOR_NORMAL);


    uint8_t status_str[20];
    //传感器是否到达温度
    if(sensorAtTemp == 0x01) {
        strcpy((char *)status_str, "Temp: Ready ");
        NOx_State=1<<5|NOx_State;
    } else if(sensorAtTemp == 0x00) {
        strcpy((char *)status_str, "Temp: Unready");

    } else if(sensorAtTemp == 0x10) {
        strcpy((char *)status_str, "Temp: no use");
    } else {
        strcpy((char *)status_str, "Temp: default");
    }

    

    
    if(voltageInRange == 0x01) {
        strcpy((char *)status_str, "voltage: InRange      ");
        NOx_State=1<<4|NOx_State;
    } else if(voltageInRange == 0x00) {
        strcpy((char *)status_str, "voltage: NotInRange ");

    } else if(voltageInRange == 0x10) {
        strcpy((char *)status_str, "voltage: no use    ");
    } else {
        strcpy((char *)status_str, "voltage: default  ");
    }



    //显示氮氧化物和氧气的16进制数据
    uint8_t row[20];
    snprintf((char *)row, sizeof(row), "O2_raw: 0x%x ", O2_raw);

    snprintf((char *)row, sizeof(row), "NOX_raw: 0x%x ", NOX_raw);

    //显示原始数据
    char hexString[2 * (J1939_MSG_LENGTH + J1939_DATA_LENGTH) + 1];
    hexArrayToString(RxMsgPtr->Array, J1939_MSG_LENGTH + J1939_DATA_LENGTH, hexString);



    NOx_State=1<<3|NOx_State;
    if(heaterControl == 0x01) {
        strcpy((char *)status_str, "heater: Heating34   ");
    } else if(heaterControl == 0x00) {
        strcpy((char *)status_str, "heater: autoHeating");
    } else if(heaterControl == 0x10) {
        strcpy((char *)status_str, "heater: Heating12  ");
    } else {
        strcpy((char *)status_str, "heater: stop        ");
        NOx_State=0x1F7 & NOx_State;
    }

    

    if(errorheater==0x1f){
       NOx_State=1<<2|NOx_State;
    }
    
    //???
    uint8_t buffer1[10];
    snprintf((char *)buffer1, sizeof(buffer1), "no%2d", errorNOx);    
    if(errorNOx==0x1f){
       NOx_State=1<<1|NOx_State; 
    }else 
       OLED_PrintASCIIString(40, 1, (char *)buffer1, &afont8x6, OLED_COLOR_REVERSED);
 
    
    snprintf((char *)buffer1, sizeof(buffer1), "o2%2d", errorO2); 
    if(errorO2==0x1f){
       NOx_State=1|NOx_State;
    }else 
       OLED_PrintASCIIString(40, 1, (char *)buffer1, &afont8x6, OLED_COLOR_REVERSED);


//    //寄存器状态更新
//    g_tVar.P01=NOx;
//    g_tVar.P02=O2;
//    //传感器状态
//    g_tVar.P07=NOx_State;
		Var_Update_SensorCore(NOx,O2,NOx_State);

}




// 接收消息任务函数
void NOxReceive(void *argument)
{
    //发送数据初始化
    TxMsg_Init(&TxMessage);
    for (;;)
    {
        
        if (pdPASS == xQueueReceive(Rx_QueueHandle, &RxMessage, 100)) {
            //消息处理
            NOx_Handle(&RxMessage);
            
            //转化为电流输出数据
            change_to_electricity(electricity_data_buf);
            
            //向从机发送命令
            
            
            //发送加热指令
            J1939_CAN_Transmit(&TxMessage);
        }
        else {
            //读取队列失败，队列空  g_tVar.P07其他数据都无效，传感器读数也无效     掉电检测
            g_tVar.P07=0x0ff & g_tVar.P07;
            OLED_PrintASCIIString(0, 10, "Not Received", &afont16x8, OLED_COLOR_NORMAL);
        }
        
        //状态显示
        uint8_t buffer[20];
        snprintf((char *)buffer, sizeof(buffer), "state: %5d ", g_tVar.P07);
        if(g_tVar.P07==0x1ff){ 
            //状态正常
            OLED_PrintASCIIString(0, 50, (char *)buffer, &afont16x8, OLED_COLOR_NORMAL);
        }
            
        else {
            //状态异常
            OLED_PrintASCIIString(0, 50, (char *)buffer, &afont16x8, OLED_COLOR_REVERSED);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//反吹到时
void Blow_Time_Out(TimerHandle_t xTimer) {
      BLOW_CONTROL(0);
      g_tVar.P24=blowspan;
}

// 默认任务函数
void NOxDefault(void *argument)
{
    char buffer[30];
    
    //创建反吹定时器
    TimerBlowback=xTimerCreate( "blowtimer",pdMS_TO_TICKS(blowtime*1000),pdFALSE,NULL,Blow_Time_Out);
    for (;;)
    {
        
        //寄存器更新
        

        //传感器标定
        Calibration_NOx(&NOx_parameter,&NOx_parameter1);
        Calibration_O2(&O2_parameter,&O2_parameter1);
        //updateConfigParam("config.txt", "NOx_A", NOx_parameter.a);

        //异常报警
        Alarm();
        
        //反吹 3600s反吹一次
        Blowback();
        
        //运行时间显示
        snprintf(buffer,sizeof(buffer),"Time:    %ds",time_1s);
        OLED_PrintASCIIString(0, 1, buffer, &afont8x6, OLED_COLOR_NORMAL);
        //OLED屏幕刷新
        OLED_ShowFrame();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




//modbus任务
void ModBusSlave(void *argument)
{

    // 创建寄存器互斥信号量（优先级继承，避免优先级反转）
    g_hVarMutex = xSemaphoreCreateMutex();
    configASSERT(g_hVarMutex != NULL);  // 确保创建成功
    BLOW_CONTROL(0);
    
    
    MDSUARTx.Init.BaudRate = SBAUD485;
    HAL_UART_Init(&MDSUARTx);
    //创建信号量
    MODRx_SemaphoreHandle=xSemaphoreCreateBinary( );
    
    //寄存器初始化
    Register_Init();
    
    //写flash 芯片第一次烧录时需要取消这个注释
    //InternalFlash_Write();
    
    //从flash读取标定参数
    LoadRegistersFromFlash();
    //将flash读取后的值更新
    AfterFlash_Init();
    
    //校准点位x值初始化
    Calibration_Init(NOx_x,NOx_y,NOx_parameter,NOx_parameter1);
    Calibration_Init(O2_x,O2_y,O2_parameter,O2_parameter1);
    
    
    //开启modbus接受中断
    Start_Receive();
    
    for(;;)
    {
        MODS_Poll();
    }

}



//标定点位初始化 ，根据y初始化x
static void Calibration_Init(uint16_t x[],float y[],Parameter p, Parameter p1){

     x[0]=(y[0]-p.b)/p.a; 
     x[1]=(y[1]-p.b)/p.a;
     x[2]=(y[2]-p1.b)/p1.a;
}

//计算斜率截距
static uint8_t Calculate_slope_intercept(uint16_t x[],float y[], Parameter* p){
    //计算斜率
    if((x[1]-x[0])==0){
        return 0;
    }
    p->a=(y[1]-y[0])/(x[1]-x[0]) ;
    //计算截距
    p->b=y[0]-p->a*x[0];
    return 1;
}

/*
*********************************************************************************************************
*	函 数 名: Blowback
*	功能说明: 反吹函数  写入0立刻反吹,反吹中寄存器状态为0,不影响定时反吹，写入0xffff立即停止反吹,写入0x01立即反吹，并重置基准时间,写入其他值设置反吹间隔
*            
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

void Blowback(void){
     
     //反吹间隔要大于反吹持续时间！
		 uint16_t Blowback_Interval=Var_Read_P24();
     if(Blowback_Interval==0xffff){
        //停止反吹
        BLOW_CONTROL(0);
        xTimerStop(TimerBlowback,portMAX_DELAY); 
        return;
     }else {
         //从寄存器中获取值
         //反吹间隔
         blowtime=Var_Read_P25();
         //获取有效值
         if(Blowback_Interval!=0 && Blowback_Interval!=1 && blowspan!=Blowback_Interval){
             blowspan=Blowback_Interval;             
             //重置基准时间
             time_1s_blow=0;
         }      

         if(Blowback_Interval==0x01){
            //立即反吹,并重置基准时间
            BLOW_CONTROL(1);
             
            //重置基准时间
            time_1s_blow=0;
         
         }else if(Blowback_Interval==0 && !blow_flag){
             //开始反吹
            BLOW_CONTROL(1);      
         }
         //定时反吹,防止上电后就反吹
         if(time_1s_blow%blowspan==0 && (time_1s_blow!=0)&&!blow_flag){           
            //开始反吹
            BLOW_CONTROL(1);
             
         }
         
         return;     
     
     }
     

}
/*
*********************************************************************************************************
*	函 数 名: Alarm
*	功能说明: 异常报警函数,NOx过高报警，O2过低报警
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void Alarm(void){
     //报警值更新
     NOx_High= Var_Read_P12();
     O2_Low = Var_Read_P13();
     if(NOx>NOx_High){
        //NOx报警
     }
     if(O2<O2_Low){
        //O2报警
     
     }
     return ;
}

//寄存器初始化(无需互斥)
void Register_Init(void){
    //寄存器报警值初始化
    g_tVar.P12=NOx_High;
    g_tVar.P13=O2_Low;
    //标定点1初始化
    g_tVar.P18=NOx_y[1];
    g_tVar.P19=O2_y[1];   
    //标定点2初始化
    g_tVar.P20=NOx_y[2];
    g_tVar.P21=O2_y[2];
    
    //转换参数存入寄存器
    g_tVar.P03=NOx_parameter.a;
    g_tVar.P04=NOx_parameter.b;
    g_tVar.P05=O2_parameter.a;
    g_tVar.P06=O2_parameter.b;
    
    g_tVar.P14=NOx_parameter1.a;
    g_tVar.P15=NOx_parameter1.b;
    g_tVar.P16=O2_parameter1.a;
    g_tVar.P17=O2_parameter1.b;
    
    g_tVar.P24=blowspan;
    g_tVar.P25=blowtime;
}

//读取寄存器后进行参数更新(无需互斥)
void AfterFlash_Init(void){
    //寄存器报警值初始化
//    g_tVar.P12=NOx_High;
//    g_tVar.P13=O2_Low;
    //标定点1初始化
    NOx_y[1]=g_tVar.P18;
    O2_y[1]=g_tVar.P19;   
    //标定点2初始化
    NOx_y[2]=g_tVar.P20;
    O2_y[2]=g_tVar.P21;
    
    //转换参数存入寄存器
    NOx_parameter.a=g_tVar.P03;
    NOx_parameter.b= g_tVar.P04;
    O2_parameter.a= g_tVar.P05;
    O2_parameter.b=g_tVar.P06;
    
    NOx_parameter1.a=g_tVar.P14;
    NOx_parameter1.b=g_tVar.P15;
    O2_parameter1.a=g_tVar.P16;
    O2_parameter1.b=g_tVar.P17;
    
    //暂时没有写入寄存器
    blowspan=g_tVar.P24;
    blowtime=g_tVar.P25;
    
}

//转换为电流输出
void change_to_electricity(uint8_t * buf){
    //NOx 0~2500ppm
    //O2 0~25%
    uint16_t nox=4000;
    uint16_t o2=4000;
    if(NOx>0){
      nox=(NOx * 16000) / 2500+4000;
    }
    if(O2>0){
      o2= (O2 * 16000) / 25+4000;
    }

    
    buf[0]= (nox>>8) &0xFF;
    buf[1]=  nox&0xFF;
    buf[2]= (o2>>8) &0xFF;
    buf[3]=  o2&0xFF;
    
}


//继电器1接换向阀
//1反吹，0不反吹
void BLOW_CONTROL(uint8_t state){
		// 1反吹，软件定时器重新定时
    if(state){
        blow_flag=1;
        Var_Write_P24(0);
				xTimerChangePeriod(TimerBlowback,pdMS_TO_TICKS(blowtime*1000),portMAX_DELAY );
    }else{
        blow_flag=0;
    }
    HAL_GPIO_WritePin(Relay0_GPIO_Port, Relay0_Pin, (state) ? GPIO_PIN_RESET : GPIO_PIN_SET); \
    HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET); \
    
    return ;
} 
