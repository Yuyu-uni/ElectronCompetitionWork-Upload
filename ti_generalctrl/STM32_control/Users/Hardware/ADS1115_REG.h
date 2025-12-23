#ifndef __ADS1115_REG_H__
#define __ADS1115_REG_H__

#define ADS1115_ADDRESS 0x48 // Default I2C address for ADS1115 (When ADDR pin connected to GND)   0100 1000

/*IIC 读写--------------------------------------------------------------------------*/
#define ADS1115_W 0x90    //1000 1000
#define ADS1115_R 0x91    //1000 1001
/*config 寄存器地址----------------------------------------------------------------------------*/
#define REG_Conversion   0x00
#define REG_config		 0x01
#define REG_L_thresh 	 0x02
#define REG_H_thresh 	 0x03
/***************/
//配置寄存器说明
//config register
/*CRH[15:8](R/W)
   BIT      15      14      13      12      11      10      9       8
   NAME     OS      MUX2    MUX1    MUX0    PGA2    PGA1    PGA0    MODE
CRL[7:0] (R/W)
   BIT      7       6       5       4       3       2       1       0
   NAME    DR0     DR1     DR0   COM_MODE COM_POL COM_LAT COM_QUE1 COM_QUE0
   -----------------------------------------------------------------------------------
 * 15    | OS             |  运行状态转换开始
 *       |                | 写时:
 *       |                | 0   : 无效
 *       |                | 1   : 开始单次转换处于掉电状态时
 *       |                | 读时:
 *       |                | 0   : 正在转换
 *       |                | 1   : 未执行转换
 * -----------------------------------------------------------------------------------
 * 14:12 | MUX [2:0]      | 输入复用多路配置
 *       |                | 000 : AINP = AIN0 and AINN = AIN1 (default)
 *       |                | 001 : AINP = AIN0 and AINN = AIN3
 *       |                | 010 : AINP = AIN1 and AINN = AIN3
 *       |                | 011 : AINP = AIN2 and AINN = AIN3
 *       |                | 100 : AINP = AIN0 and AINN = GND
 *       |                | 101 : AINP = AIN1 and AINN = GND
 *       |                | 110 : AINP = AIN2 and AINN = GND
 *       |                | 111 : AINP = AIN3 and AINN = GND
 * -----------------------------------------------------------------------------------
 * 11:9  | PGA [2:0]      | 可编程增益放大器配置(FSR  full scale range) (量程)
 *       |                | 000 : FSR = ±6.144 V
 *       |                | 001 : FSR = ±4.096 V
 *       |                | 010 : FSR = ±2.048 V (默认)
 *       |                | 011 : FSR = ±1.024 V
 *       |                | 100 : FSR = ±0.512 V
 *       |                | 101 : FSR = ±0.256 V
 *       |                | 110 : FSR = ±0.256 V
 *       |                | 111 : FSR = ±0.256 V
 * -----------------------------------------------------------------------------------
 * 8     | MODE           | 工作模式
 *       |                | 0   : 连续转换
 *       |                | 1   : 单次转换
 * -----------------------------------------------------------------------------------
 * 7:5   | DR [2:0]       | 采样频率
 *       |                | 000 : 8 SPS
 *       |                | 001 : 16 SPS
 *       |                | 010 : 32 SPS
 *       |                | 011 : 64 SPS
 *       |                | 100 : 128 SPS (默认)
 *       |                | 101 : 250 SPS
 *       |                | 110 : 475 SPS
 *       |                | 111 : 860 SPS
 * -----------------------------------------------------------------------------------
 * 4     | COMP_MODE      | 比较器模式
 *       |                | 0   : 传统比较器 (default)
 *       |                | 1   : 窗口比较器
 * -----------------------------------------------------------------------------------
 * 3     | COMP_POL       | Comparator polarity
 *       |                | 0   : 低电平有效 (default)
 *       |                | 1   : 高电平有效
 * -----------------------------------------------------------------------------------
 * 2     | COMP_LAT       | Latching comparator
 *       |                | 0   : 非锁存比较器. (default)
 *       |                | 1   : 锁存比较器.
 * -----------------------------------------------------------------------------------
 * 1:0   | COMP_QUE [1:0] | Comparator queue and disable
 *       |                | 00  : Assert after one conversion
 *       |                | 01  : Assert after two conversions
 *       |                | 10  : Assert after four conversions
 *       |                | 11  : 禁用比较器并将ALERT/RDY设置为高阻抗 (default)
 * -----------------------------------------------------------------------------------
*/
/* REG_config Bit15-0 ------------------------------------------------------------------*/
#define OS 				     1 		  //操作状态或单发转换启动  （1位）
#define MUX 			     0x04 	  //输入多路配置  通道0  （3位）
#define PGA 			     0x00	  //可编程增益放大器配置  量程 ±6.144  V  （3位）
#define MODE		         0x00	  //设备运行方式  连续转换模式  （1位）
#define DR				     0x03     //转换速率920SPS  （3位）
#define	COMP_MODE	      0 		  //比较器模式  传统比较器  （默认）（1位）
#define COMP_POL 	         0		  //比较器极性  低电平有效  （默认）（1位）
#define COMP_LAT	         0 		  //锁存比较器  非锁存比较器， 置位后ALERT / RDY引脚不锁存  （默认）（1位）
#define COMP_QUE	         0x03 	  //比较器置位和禁用  禁用 （默认）（2位）


#define config_MSB (OS << 7)|(MUX << 4)|(PGA << 1)|(MODE)
#define config_LSB (DR << 5)|(COMP_MODE << 4)|(COMP_POL << 3)|(COMP_LAT << 2)|(COMP_QUE)


#endif
