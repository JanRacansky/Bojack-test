#define TMC2209_R_GCONF_RW     0x00 
union tmc2209_gconf {
    uint32_t d;
    struct {
        uint i_scale_analog : 1;    // 0=internal, 1=VREF
        uint internal_rsense : 1;   // 0=external resistors, 1=internal resistor
        uint en_spreadCycle : 1;    // 0=StealthChop, 1=SpreadCycle
                                    // A high level on the pin SPREAD inverts this flag
        uint shaft : 1;             // 1=inverse motor direction
        uint index_otpw : 1;        // 0=INDEX shows first microstep position, 1=INDEX shows overtemperature
        uint indes_step : 1;        // 0=INDEX  used as defined by index_otpw, 1=INDEX shows steps pulses
        uint pdn_disable : 1;       // 0=PDN_UART pin controls power down, 1=PDN_UART pin used for UART
        uint mstep_reg_select : 1;  // 0=microsteps defined by MS1,2 pisn, 1=microsteps defined by MRES register
        uint multistep_filt : 1;    // 0=no steps pulse optimization, 1=software step pulse optimisation
        uint test_mode : 1;         // 0=normal mode, 1=factory testing
    };
};

#define TMC2209_R_CHOPCONG_RW  0x6C
union tmc2209_chopconf {
    uint32_t d;
    struct
    {
        uint toff : 4;      // dulration of slow decay phase in clk = 24 + 32*toff
                            // 0=driver disabled, 1=use only with TBL>2, 2..15
        uint hstrt : 3;     // adds 1..8 clk to hysteresis low value hend
        uint hend : 4;      // hysteresis -3,-2,-1,0,1, .. ,12
        uint blank : 4;     // set to 0
        uint tbl : 2;       // comparator blank time 0b00=16, 0b01=24, 0b10=32, 0b11=40 clk
        uint vsense : 1;    // 0=low sensitivity, 1=high sensitivity
        uint blank1: 6;     // set to 0
        uint mres : 4;      // 0b0000=256 micro steps, 0b0001=128 micro steps, ... 0b1000=full step
        uint intpol : 1;    // 1 = mres exptrapolated for 256 microsteps for smooth movement
        uint dedge : 1;     // 1 = step at each edge of STEP pin signal 
        uint diss2g : 1;    // 1 = disable short to GND protection 
        uint diss2v : 1;    // 1 = disable short protection on low side
    };
};

#define TMC2209_R_IHOLD_IRUN_W     0x10 
union tmc2209_ihold_irun {
    uint32_t d;
    struct 
    {
        uint ihold : 5;     // standstill current 0=1/32,..,31=32/32
        uint blank : 3;
        uint irun : 5;      // motor run current 0=1/32,..,31=32/32
        uint blqnk1 : 3;
        uint iholddelay : 4;    // number of clock for power down after staandstill 0=instant, n*2^18
    };
};

#define TMC2209_R_TPOWERDOWN_W  0x11
union tmc2209_tpowerdown {
    uint32_t d;
    struct 
    {
        uint tpowerdown : 8;    // time from standstill detection to power down in n*2^18 clk
    };
};

#define TMC2209_R_PWMCONF_RW    0x70
union tmc2209_pwmconf {
    uint32_t d;
    struct {
        uint pwm_ofs : 8;       // PWM amplitude relative to full motor current
                                // 0 = disable scaling down, >0 = scaling down enabled
        uint pwm_grad : 8;      // velocity dependent gradient for PWM amplitude
        uint pwm_freq : 2;      // 0 = 2/1024 clk, 1 = 2/683 clk, 2 = 2/512 clk, 3 = 2/410 clk
        uint pwm_autoscale : 1; // 1 = Enable automatic current control (Reset default)
        uint pwm_autograd : 1;  // 1 = Automatic tuning (only with pwm_autoscale=1) 
        uint freewheel : 2;     // 0 = Normal operation, 1 = Freewheeling, 2 = Coil shorted using LS drivers, 3: Coil shorted using HS drivers
        uint blank : 2;
        uint pwm_reg : 4;       // maximum pwm amplitude change per half wave in autoscale mode, n=pwm_reg/2 increments
        uint pwm_lim : 4;       // Limit for PWM_SCALE_AUTO when switching back from SpreadCycle to StealthChop. 
                                // This value defines the upper limit for bits 7 to 4 of the automatic current control when switching back.
    };
};

#define TMC2209_R_IOIN      0x06
union tmc2209_ioin {
    uint32_t d;
    struct {
        uint enn : 1;
        uint blank : 1;
        uint ms1 : 1;
        uint ms2 : 1;
        uint diag : 1;
        uint blank1 : 1;
        uint pdn_uart : 1;
        uint step : 1;
        uint spread_enn : 1;
        uint dir : 1;
        uint blank3 : 14;
        uint version : 8;       // should be always 0x21
    };
};