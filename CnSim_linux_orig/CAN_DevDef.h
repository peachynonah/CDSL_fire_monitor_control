#include <stdint.h>

// Type Definition
typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef uint32_t		UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;

// Type Definition Structure
typedef struct {
	UINT32 COB_ID;
	UINT8 LD;				// Byte[0] -> Legend Data(LD)
	UINT16 Index;			// Byte[1,2]
	UINT8 SubIndex;			// Byte[3]
	INT32 Value;			// Byte[4,5,6,7]
}SDO_Frame;

// Node ID
#define P_ID			0x01 // Pan ID
#define T_ID			0x02 // Tilt ID

#define FL_F_ID			0x01 // Front Left Flipper ID
#define FR_F_ID			0x02 // Front Right Flipper ID
#define RL_F_ID			0x03 // Rear Left Flipper ID
#define RR_F_ID			0x04 // Rear Right Flipper ID

// CAN SDO Communication
#define SDO_W_ID		0x600 // Service Data Object Write ID
#define SDO_R_ID		0x580 // Service Data Object Read ID

// CAN Legend Data Bit
#define LD_CCS_R	   	0x40 // Client Command Specifier(CCS, Bit 7...5) Read Object = 2(010)
#define LD_CCS_W	  	0x20 // Client Command Specifier(CCS, Bit 7...5) Write Object = 1(001)
#define LD_SCS_R	   	0x40 // Servo Command Specifier(SCS, Bit 7...5) Read Object = 2(010)
#define LD_SCS_W	  	0x60 // Servo Command Specifier(SCS, Bit 7...5) Write Object = 3(011)
#define LD_CS		  	0x80 // Command Specifier(CS, Bit 7..5) SDO Abort Transfer = 4
#define LD_N_1B	  		0x0C // Number of Byte(Bit 3..2)
#define LD_N_2B	  		0x08 // Number of Byte(Bit 3..2)
#define LD_TT		  	0x02 // Transfer Type(Bit 1)
#define LD_SI	      	0x01 // Size Indicator(Bit 0)

// CAN PDO Communication

// Slave(Dev)->Master(PC)
#define TxPDO_I			0x1800 // TxPDO Index

// Master(PC)->Slave(Dev)
#define RxPDO_I			0x1400 // RxPDO Index

// Config PDO Subindex
#define PDO_S_S				0x00 // PDO Status Subindex
#define PDO_COB_ID_S		0x01 // PDO COB-ID Subindex
#define PDO_TT_S 			0x02 // PDO Transfer Type Subindex

// Config PDO Transfer Type
#define PDO_TT_SYNC			0x01 // Syncronous
#define PDO_TT_RTR			0xFD // Remote Transmission Request(RTR)
#define PDO_TT_ASYNC		0xFF // Asyncronous

// COB-ID Sync
#define COB_ID_SYNC			0x80 // Communication Object IDentifier of the SYNC

// NMT Protocol(Network ManagemenT Protocol : COB-ID|Command Specifier(CS)|NODE_ID)
#define NMT_COB_ID			0x00 // NMT COB-ID
#define NMT_CS_PRE_OP		0x80 // NMT CS Pre-Operation
#define NMT_CS_RST_DEV		0x81 // NMT CS RESET DEVICE
#define NMT_CS_RST_COM		0x82 // NMT CS RESET COMMUNCATION
#define NMT_CS_OP			0x01 // NMT CS Operation
#define NMT_CS_STOP			0x02 // NMT CS Stop

// NMT Heartbeat State
#define NMT_HS_ID		0x700 // NMT Heartbeat State COB_ID = NMT_HB_ID + NODE_ID,If Node_ID = 0x01, COB_ID = 0x701
#define NMT_HS_BOOT_UP	0x00  // NMT Heartbeat State : Bootup
#define NMT_HS_STOP		0x04  // NMT Heartbeat State : Stopped
#define NMT_HS_PRE_OP	0x7F  // NMT Heartbeat State : Pre-Operational
#define NMT_HS_OP		0x05  // NMT Heartbeat State : Operational
#define NMT_HS_NONE     0xFF  // NMT Heartbeat State : None(User Define)

//NMT Producer heartbeat time
#define NMT_HB_Time_I   0x1017 // NMT Producer Heartbeat Time Index
#define NMT_HB_Time_S   0x00   // NMT Producer Heartbeat Time Subindex

// Object 0x6040 : Control Word
#define CTRL_I								0x6040 // Control Word Index
#define CTRL_S								0x00   // Control Word Subindex
#define CTRL_SWT_ON							0x0001 // Bit 0
#define CTRL_ENA_VOLT						0x0002 // Bit 1
#define CTRL_QUICK_STOP						0x0004 // Bit 2
#define CTRL_ENA_OP							0x0008 // Bit 3
#define CTRL_OPMODE_NEW_SETPOINT			0x0010 // Bit 4
#define CTRL_OPMODE_CHN_SET_IMT				0x0020 // Bit 5
#define CTRL_OPMODE_ABS_OR_REL				0x0040 // Bit 6
#define CTRL_FAULT_RESET					0x0080 // Bit 7
#define CTRL_OPMODE_HALT					0x0100 // Bit 8
#define CTRL_OPMODE_ENDLESS_MOVE			0x8000 // Bit 15

// Controller Command
// #define CTRL_CMD_MASK		  0x008F // Control Commands Mask
// #define CTRL_CMD_SHUTDOWN	  0x0006 // Shutdown (Transition 2, 6, 8) */
// #define CTRL_CMD_SWITCH_ON	  0x0007 // Switch On (Transition 3)
// #define CTRL_CMD_ENABLE_OP	  0x000F // Switch On (Transition 3)

// Object 0x6041 : Status Word - The Drive State & MODE Specified(PPM. PVM, HMM, CSP, CSV, CST)
#define STAT_I			 					0x6041 // Status Word Index	
#define STAT_S			 					0x00   // Status Word Subindex
#define STAT_NOT_RDY_TO_SWT_ON				0x0000 // All Bits are zero.
#define STAT_RDY_TO_SWT_ON					0x0001 // Bit 0
#define STAT_SWT_ON							0x0002 // Bit 1
#define STAT_OP_ENA							0x0004 // Bit 2
#define STAT_FAULT							0x0008 // Bit 3
#define STAT_VOLT_ENA						0x0010 // Bit 4
#define STAT_QUICK_STOP						0x0020 // Bit 5
#define STAT_SWT_ON_DIS						0x0040 // Bit 6
#define STAT_WARNING						0x0080 // Bit 7
#define STAT_RESERVED_1						0x0100 // Bit 8
#define STAT_REMOTE							0x0200 // Bit 9
#define STAT_RESERVED_3						0x4000 // Bit 14
#define STAT_POS_REF_TO_HOME_POS			0x8000 // Bit 15
#define STAT_TARGET_REACHED					0x0400 // Bit 10 for PPM or PVM or HMM 
#define STAT_RESERVED_2						0x0400 // Bit 10 for CSP or CSV or CST
#define STAT_INT_LIM_ACT					0x0800 // Bit 11 for PPM or HMM or CSP or CSV or CST
#define STAT_SPD_LIM						0x0800 // Bit 11 for PVM
#define STAT_SET_POINT_ACK					0x1000 // Bit 12 for PPM
#define STAT_SPD							0x1000 // Bit 12 for PVM
#define STAT_HOMING_ATTAINED				0x1000 // Bit 12 for HMM
#define STAT_FOLLOW_CMD_VAL					0x1000 // Bit 12 for CSP or CSV or CST
#define STAT_FOLLOW_ERR						0x2000 // Bit 13 for PPM or CSP
#define STAT_HOMMING_ERR					0x2000 // Bit 13 for HMM
#define STAT_NOT_USED						0x2000 // Bit 13 for PVM or CSV or CST

// #define DRV_RDY_TO_SWT_ON	  0x21 // Ready to switch on     : Status Word x01x 0001 */
// #define DRV_SWT_ON			  0x23 // Switched on            : Status Word x011 0011 */
// #define DRV_OP_ENA			  0x27 // Operation enabled      : Status Word x011 0111 */
// #define DRV_QUICK_STOP		  0x07 // Quick stop active      : Status Word 0000 0111 */
// #define DRV_MALFNC_REA		  0x0F // Malfunction Reaction Active
// #define DRV_MALFNC			  0x08 // Malfunction

// Object 0x6060 : Modes of operation
#define MO_I								0x6060
#define MO_S								0x00

// Object 0x6061 : Modes of operation display (RO)
#define MODIS_I							0x6061
#define MODIS_S							0x00

// Modes
#define PPM								0x01
#define PVM								0x03
#define HMM								0x06
#define CSP								0x08
#define CSV								0x09
#define CST								0x0A

// Object 0x3150 : Digital output properties
#define DO_PROP_I							0x3150 // Digital Output Properties Index
#define DO_LS_S								0x01   // Digital Output Logic State Subindex(RO)
#define DO_PO_S								0x02   // Digital Output Polality Subindex(RW)

// Object 0x3151 : Configuration of digital outputs
#define DO_CONF_I							0x3151  // Digital Output Configuration Index
#define DO_1_CONF_S							0x01    // Digital Output 1 Configuration Subindex
#define DO_2_CONF_S							0x02    // Digital Output 2 Configuration Subindex
#define DO_HS_CONF_S						0x03    // Digital Output High Speed Configuration Subindex
#define DO_CONF_SET_BK						0x00    // Value : 0
#define DO_CONF_GPIO_A						0x20 	// Value : 16
#define DO_CONF_GPIO_B						0x21 	// Value : 17
#define DO_CONF_GPIO_C						0x22 	// Value : 18
#define DO_CONF_HOLD_BK						0x30	// Value : 24
#define DO_CONF_RDY_OR_FLT					0x31	// Value : 25
#define DO_CONF_NONE						0xFF	// Value : 255

// Object 0x60FE : Digital Outputs
// Read and Write
#define DO_I							0x60FE // Digital Output Index
#define DO_S							0x00 // Digital Output Subindex
#define DO_SET_BRAKE					0x00000000 // Bit 0
#define DO_SET_GPIO_A					0x00010000 // Bit 16
#define DO_SET_GPIO_B					0x00020000 // Bit 17
#define DO_SET_GPIO_C					0x00040000 // Bit 18
#define DO_SET_NONE						0x00040000 // Bit 18
// Read Only
#define DO_HOLDING_BRAKE				0x01000000 // Bit 24 Output functionality to drive a holding brake
#define DO_READY_OR_FAULT				0x02000000 // Bit 25 Active on device ready / inactive on device fault state

// Object 0x603F : Error code(Last Error)
#define ErrCode_I						0x603F // Error Code Index
#define ErrCode_S						0x00   // Error Code Subindex

// Object 0x1000 : Device type
// Object 0x1001 : Error register
// Object 0x1003 : Error history 
// Object 0x1005 : COB - ID SYNC 
// Object 0x1008 : Manufacturer device name
// Object 0x1010 : Store parameters
// Object 0x1011 : Restore default parameters
// Object 0x1014 : COB - ID EMCY 
// Object 0x1016 : Consumer heartbeat time 
// Object 0x1017 : Producer heartbeat time 
// Object 0x1018 : Identity object 
// Object 0x1029 : Error behavior 
// Object 0x10F3 : Diagnosis History 
// Object 0x1200 : SDO server parameter 
// Object 0x1400 : Receive PDO 1 parameter 
// Object 0x1401 : Receive PDO 2 parameter 
// Object 0x1402 : Receive PDO 3 parameter 
// Object 0x1403 : Receive PDO 4 parameter 
// Object 0x1600 : Receive PDO 1 mapping 
// Object 0x1601 : Receive PDO 2 mapping 
// Object 0x1602 : Receive PDO 3 mapping 
// Object 0x1603 : Receive PDO 4 mapping 
// Object 0x1800 : Transmit PDO 1 parameter 
// Object 0x1801 : Transmit PDO 2 parameter 
// Object 0x1802 : Transmit PDO 3 parameter 
// Object 0x1803 : Transmit PDO 4 parameter 
// Object 0x1A00 : Transmit PDO 1 mapping 
// Object 0x1A01 : Transmit PDO 2 mapping 
// Object 0x1A02 : Transmit PDO 3 mapping 
// Object 0x1A03 : Transmit PDO 4 mapping 
// Object 0x1C00 : SYNC manager communication type 
// Object 0x1C12 : SYNC manager 2 PDO assignment 
// Object 0x1C13 : SYNC manager 3 PDO assignment 
// Object 0x1C32 : SYNC manager 2 parameter 
// Object 0x1C33 : SYNC manager 3 parameter 
// Object 0x1F50 : Program data
// Object 0x1F51 : Program control
// Object 0x1F56 : Program software identification
// Object 0x1F57 : Flash status identification
// Object 0x2000 : Node - ID
// Object 0x2001 : CAN bit rate 
// Object 0x2002 : RS232 bit rate
// Object 0x2005 : RS232 frame timeout
// Object 0x2006 : USB frame timeout
// Object 0x200A : CAN bit rate display 
// Object 0x2010 : Active fieldbus
// Object 0x2100 : Additional identity 
// Object 0x2101 : Extension 1 identity 
// Object 0x210C : Custom persistent memory
// Object 0x2200 : Power supply 
// Object 0x2201 : Power supply supervision 
// Object 0x3000 : Axis configuration 
// Object 0x3001 : Motor data 
// Object 0x3002 : Electrical system parameters 
// Object 0x3003 : Gear configuration 
// Object 0x3010 : Digital incremental encoder 1 
// Object 0x3011 : Analog incremental encoder 
// Object 0x3012 : SSI absolute encoder 
// Object 0x301A : Digital Hall sensor 
// Object 0x3020 : Digital incremental encoder 2 
// Object 0x30A0 : Current control parameter set 
// Object 0x30A1 : Position control parameter set 
// Object 0x30A2 : Velocity control parameter set 
// Object 0x30A3 : Velocity observer parameter set 
// Object 0x30AE : Dual loop position control parameter set 
// Object 0x30B0 : Home position
// Object 0x30B1 : Home offset move distance
// Object 0x30B2 : Current threshold for homing mode
// Object 0x30D0 : Current demand value
// Object 0x30D1 : Current actual values
// Object 0x30D2 : Torque actual values
// Object 0x30D3 : Velocity actual values
// Object 0x30E0 : Standstill window configuration 
// Object 0x3141 : Digital input properties
// Object 0x3142 : Configuration of digital inputs
// Object 0x3158 : Holding brake parameters 
// Object 0x3160 : Analog input properties
// Object 0x3161 : Configuration of analog inputs
// Object 0x3162 : Analog input general purpose
// Object 0x3163 : Analog input adjustment 
// Object 0x3170 : Analog input current set value properties
// Object 0x3171 : Analog input velocity set value properties
// Object 0x3180 : Analog output properties
// Object 0x3181 : Configuration of analog outputs
// Object 0x3182 : Analog output general purpose
// Object 0x3200 : Power limitation 
// Object 0x3201 : Thermal overload protection 
// Object 0x3202 : Functional safety 
// Object 0x3203 : Motor control 
// Object 0x6007 : Abort connection option code
// Object 0x603F : Error code(Last Error)
// Object 0x605A : Quick stop option code
// Object 0x605B : Shutdown option code
// Object 0x605C : Disable operation option code
// Object 0x605E : Fault reaction option code
// Object 0x6062 : Position demand value
// Object 0x6064 : Position actual value
// Object 0x6065 : Following error window
// Object 0x6066 : Following error time out
// Object 0x6067 : Position window
// Object 0x6068 : Position window time
// Object 0x606B : Velocity demand value
// Object 0x606C : Velocity actual value
// Object 0x6071 : Target torque
// Object 0x6076 : Motor rated torque
// Object 0x6077 : Torque actual value
// Object 0x607A : Target position
// Object 0x607B : Position range limit
// Object 0x607D : Software position limit
// Object 0x607F : Max profile velocity
// Object 0x6080 : Max motor speed
// Object 0x6081 : Profile velocity
// Object 0x6083 : Profile acceleration
// Object 0x6084 : Profile deceleration
// Object 0x6085 : Quick stop deceleration
// Object 0x6086 : Motion profile type
// Object 0x6098 : Homing method
// Object 0x6099 : Homing speeds
// Object 0x609A : Homing acceleration
// Object 0x60A8 : SI unit position
// Object 0x60A9 : SI unit velocity
// Object 0x60AA : SI unit acceleration
// Object 0x60B0 : Position offset
// Object 0x60B1 : Velocity offset
// Object 0x60B2 : Torque offset
// Object 0x60B8 : Touch probe function
// Object 0x60B9 : Touch probe status
// Object 0x60BA : Touch probe 1 positive edge
// Object 0x60BB : Touch probe 1 negative edge
// Object 0x60C2 : Interpolation time period 
// Object 0x60C5 : Max acceleration
// Object 0x60D0 : Touch probe source
// Object 0x60D5 : Touch probe 1 positive edge counter
// Object 0x60D6 : Touch probe 1 negative edge counter
// Object 0x60E3 : Supported homing methods
// Object 0x60E4 : Additional position actual values
// Object 0x60E5 : Additional velocity actual values
// Object 0x60F4 : Following error actual value
// Object 0x60FD : Digital inputs
// Object 0x60FE : Digital outputs
// Object 0x60FF : Target velocity
// Object 0x6402 : Motor type
// Object 0x6502 : Supported drive modes
// Object 0xF000 : Modular device profile 
// Object 0xF030 : Configured module ident list 
// Object 0xF050 : Detected module ident list ARRAY