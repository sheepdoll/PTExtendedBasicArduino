;
;	SOL Terminal VDM glass teletype emulator 
;	-- with Basic interpreter
;*
;*
;*
;*
;*
;*
;*     ****     ***     ***    *****    ***
;*     *   *   *   *   *   *     *     *   *
;*     *   *   *   *   *         *     *
;*     ****    *****    ***      *     *
;*     *   *   *   *       *     *     *
;*     *   *   *   *   *   *     *     *   *
;*     ****    *   *    ***    *****    ***
;*
;*
;*
;*
;*
;*  COPYRIGHT (c) 1977, 1978, 1979  PROCESSOR TECHNOLOGY CORP.
;*        7100 JOHNSON INDUSTRIAL DR.
;*        PLEASANTON, CALIF. 94566
;*
;*       << ALL RIGHTS RESERVED >>
;*
;*
;*
;*
;*
.include "m328def.inc"


.equ kBaud =  ((F_CPU / 4 / baud) - 1) / 2


; note pins 14 RS/DE and 16 RD are not connected at header
; this is why we need to use 3 wire mode

;	pins are: (graphic is Ic pin layout)
	
;	01 RST  6		5 ADDR0	28	SCK
;	02 RXD  0		4 ADDR1	27	SDA
;	03 TXD  1		3 ADDR2	26
;	04 IND1	2		2 ADDR3	25
;	05 IND2	3       1 ADDR4	24
;	06 OEn	4		0 ADDR5	23
;	07 VCC			  GND	22 
;	08 GND			  AREF	21
;	09 XTL			  AVCC	20
;	10 XTL			5 OPT2	19	SCL	
;	11 SRCK	5		4 OPT3	18	MISO
;	12 RCK	6		3 OPT4	17	MOSI
;	13 SRD	7		2 SDSS	16
;	14 INDR	0		1 INDG	15

;	Latches, Chip Selects, Enables

; TD-T320 pin definitions
;
; 1      2
; GND    LED_A
; LED_K1 LED_K2
; LED_K3 LED_K4
; LED_K5 VCI
; VCI    GND
; GND    IO/VCC
; CS     nc
; WR/SCL nc
; SDA    DB0
; ... data bus 1 .. 16
; 35     36
; DB17   DEN
; PCLK   HSYNC
; VSYNC  RES
; GND

; define software SPI for SD card use

.equ DD_MOSI 	= DDB3
.equ DD_SCL		= DDB5
.equ DDR_SPI 	= DDRB

.equ SS_DD		= DDRD
.equ DD_SS		= DDD5
.equ SS_PORT	= PORTD
.equ SS			= PORTD5

.equ DC_DD		= DDRD
.equ DD_DC		= DDD6
.equ DC_PORT	= PORTD
.equ DC			= PORTD6


; define TFT reset line for software control

.equ DD_RSTTFT	= DDRB
.equ RSTTFT_DD	= DDB1
.equ PORT_RSTTFT = PORTB
.equ RSTTFT		= PORTB1

; convenience macros for shift register data mode

.equ DD_DATA	= DDRB
.equ SR_PORT	= PORTB
.equ SR_DATA	= PORTB

.equ SR_OUT		= PORTB3	; this is equivelent to MOSI
.equ DD_OUT		= DDB3	

; Soft SPI clock line  typically orange wire (sometimes blue)
.equ SR_SCL		= PORTB5; also called SCK, CLOCK, SRCK


.macro _XCHG
	movw DPL,ZL
	movw ZL,YL
	movw YL,DPL
.endmacro

.macro _DSUB
	sub ZL,YL
	sbc ZH,YH
.endmacro

;*
;*     STORE DE FROM ADDRESS IN HL, AND INC HL BY 2
;*
.macro _DSTOR
    st      Z+,YL
    st      Z+,YH
.endmacro

;*
;*     LOAD DE FROM ADDRESS IN HL, AND INC HL BY 2
;*
.macro _DLOAD
    ld      YL,Z+
    ld      YH,Z+
.endmacro


;*
;*     COMPARE HL TO DE
;*       HL-DE
;*
.macro _HDCMP    
	cp ZL,YL
	cpc ZH,YH
.endmacro

.macro _DAD_SP
; get the stack pointer into a temp variable
in ARGL,SPL
in ARGH,SPH
; now do the add (which is probably a subtract)
add ZL,ARGL
adc ZH,ARGH
.endmacro

.macro _INR_M
	ld ARGL,Z
	inc ARGL
	st Z,ARGL
.endmacro

.macro _DCR_M
	ld ARGL,Z
	dec ARGL
	st Z,ARGL
.endmacro

.macro _INX_SP
	pop c_tmp
.endmacro

.listmac

; rename registers
; r0 r1 shadowed direct
;.def 			= r2
;.def 			= r3	
;.def 			= r4	
.def DPL			= r6	; saved pointer into ram
.def DPH			= r7	; can be used for swapping
;r8	could use for scroll index
;r9
;.def 				= r10	; internal hardware cursor pair
;.def 				= r11	;
;r12						; scroll buffer pair
;r13 13h					
.def zero			= r14
.def ssreg    		= r15

; R16 -- use for milisecond delay timer can be direct loaded
; R17 -- high part of simple milisecond delay timer
.def oDATA	  		= r18	; used to sim xram bank select
; r19
.def c_tmp    		= r20 	; compare and exchange temp 
.def idx			= r21	; quick index counter	

.def ARGL     		= r22 	; passed arguments
.def ARGH     		= r23 	; to OS, usually a pointer

; Register overlay for 8080 type code
.def A      		= r24 	; 

; TFT DBI Type-C - option 1 requires 9 bits
.equ  SR_OUTPUTS	= 9

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

;


.equ	TIMCONST	= 802	; TIME CONSTANT FOR PAUSE
; IF SOLOS
.equ	ITIMCONST	= 777	; TIME CONSTANT FOR INPUT
; ENDF

.equ	OFFSET	= 45  		; THE KEYS TO THE FRONT DOOR
;*
;*
;*      THE STACK
;*
.equ SSIZE	= 150  			; STACK SIZE
;*
;*     FLOATING POINT
;*
;*  PRECISION MAY BE 6, 8, 10, 12, 14, OR 16 ONLY!
;*
.equ	PRECISION	= 8					; NUMBER OF DIGITS OF PRECISION.
.equ	FPSIZ		= PRECISION/2+2		; BYTES IN AN FP NUMBER
.equ	FPBYT		= FPSIZ-2			; BYTES OF COEFFICIENT
.equ	DIGIT		= FPBYT				; DITTO
.equ	FPNIB		= FPBYT*2			; NUMBER OF DIGITS IN FP NUMBER
;*
;*  SET UP THE PRECISION INDICATORS
;*  IF PRECISION IS NOT RIGHT FOR A PARTICULAR PAIR OF
;*  EQUATES BELOW THEN 1 OF THE 2 DIVISIONS WILL BE 0 AND
;*  THE MULTIPLY RESULT WILL BE 0. ELSE THE RESULT IS 1.
;*
.equ	TMP6	= PRECISION/6
.equ	PX6	= 6/PRECISION*TMP6
;*
.equ	TMP8	= PRECISION/8
.equ	PX8	= 8/PRECISION*TMP8
;*
.equ	TMP10	= PRECISION/10
.equ	PX10	= 10/PRECISION*TMP10
;*
.equ	TMP12	= PRECISION/12
.equ	PX12	= 12/PRECISION*TMP12
;*
.equ	TMP14	= PRECISION/14
.equ	PX14	= 14/PRECISION*TMP14
;*
.equ	TMP16	= PRECISION/16
.equ	PX16	= 16/PRECISION*TMP16
;*
.equ	TMP40	= PRECISION/40
.equ	PX40	= 40/PRECISION*TMP40
;*
;*
;*
;*     I/O BUFFERS
;*
.equ	LINMAX	= 72		;CHARACTERS PER INPUT RECORD (133)
;*  FOR WMAX, PREC/3 IS FOR COMMAS, 15 IS FOR EXP,SIGN,$,.,+ SOME
.equ	WMAX	= PRECISION/3+PRECISION+15  ;THE MAX WIDTH OF AN FP NUMBER
;*
;*     CHARACTERS
;*
.equ 	CR		= 0x0D
.equ	NULL	= 0x00
.equ	BS		= 0x7F
.equ	LF		= 0x0A
.equ	KBRK	= 0x00		; this is the break char sent by the terminal
.equ	EOF		= 0x01
.equ	KCAN	= 0x18		; ^X	^V makes more sense for insert
.equ	KION	= 0x17		; ^W
.equ	KIOFF	= 0x1A		; ^Z
.equ	KRIGHT	= 0x0C		; ^L or ^S ^S is handshake word
.equ	KLEFT	= 0x01		; ^A or ^H
.equ	KLF		= 0x0A		; ^J
.equ	KESC	= 0x1B		; ^[


.equ #RUB  = 0x7F			; rubout character
.equ #FILL = 0x80			; 200 - filler bytes
.equ #POS  = 0x81			; 201 - position cursor
.equ #RET  = 0x82			; 202 - carriage return
.equ #CLS  = 0x83			; 203 - clear screen
.equ #EOS  = 0x84			; 204 - erase to end-of-screen
.equ #EOL  = 0x85			; 205 - erase to end-of-line
.equ #BOX  = 0x86			; 206 - draw box
.equ #CON  = 0x87			; 207 - cursor on
.equ #COF  = 0x88			; 210 - cursor off
.equ #HOM  = 0x89			; 211 - cursor home
.equ #RFS  = 0x8A			; 212 - refresh screen
.equ #UP   = 0x8B			; 213 - cursor up
.equ #DWN  = 0x8C			; 214 - cursor down
.equ #FWD  = 0x8D			; 215 - cursor forward
.equ #BAK  = 0x8E			; 216 - cursor back

;IF PTDOS
;COPY NPTDEFS/0
;.equ	SYSIN	= CONIN
;.equ	SYSTS	= CONTST
;ENDF
;IF SOLOS
;.equ	SYSIN	= 0xC01F
;.equ	SYSOT	= 0xC019
;.equ	SYSTS	= SYSIN
;.equ	AOUT	= 0xC01C
; ENDF
;*
;*
.equ	NTYPE 	= 0*16
.equ	STYPE 	= 1*16
.equ	NFTYPE	= 2*16
.equ	SFTYPE	= 3*16
.equ	MTYPE 	= 4*16
.equ	COMVD 	= 1*128
;*
;*
;*     THE CONTROL STACK  (CS)
;*
.equ	STESIZ	= 2+FPSIZ
.equ	FORSZ	= FPSIZ*2+2+2+1
.equ	ETYPE	= 0
.equ	FRTYPE	= ETYPE+1
.equ	GTYPE	= FRTYPE+1
.equ	ANTYPE	= GTYPE+1
.equ	EFTYPE	= ANTYPE+1
.equ	SBTYPE	= EFTYPE+1
.equ	FNTYPE	= NFTYPE
.equ	FSTYPE	= SFTYPE
;*
;*
;*

;*
;*
;*     TOKEN DEFFINITIONS
;*
;*     A TOKEN IS STRUCTURED AS FOLLOWS:
;*  BIT       USE
;*   7     1=TOKEN, 0=NON-TOKEN-BYTE
;*  6-0    TOKEN IDENTIFIER (0-127)
;*
.equ TOKEN = 0x80
;*
;*
;*    STATEMENT TOKEN DEFINITIONS
;*
	.equ STEPRW = TOKEN+0  ;STEP
	.equ TORW = STEPRW+1  ; TO
	.equ ELSERW = TORW+1  ; ELSE
	.equ THENRW = ELSERW+1  ; THEN
	.equ FNRW = THENRW+1  ; FN
	.equ TABRW = FNRW+1  ; TAB
	.equ CHRRW = TABRW+1  ; CHR
	.equ ASCRW = CHRRW+1  ; ASC
	.equ ERRRW = ASCRW+1  ; ERR
	.equ VALRW = ERRRW+1  ; VAL
	.equ STRRW = VALRW+1  ; STR
	.equ ZERRW = STRRW+1  ; MAT ZER
	.equ CONRW = ZERRW+1  ; MAT CON
	.equ IDNRW = CONRW+1  ; MAT IDN
	.equ INVRW = IDNRW+1  ; MAT INV
	.equ TRNRW = INVRW+1  ; MAT TRN
	.equ SELRW = TRNRW+1  ; LL=
	.equ SEMRW = SELRW+1  ; ML=
	.equ SE1RW = SEMRW+1  ; DS=
; IF SOLOS
	.equ SE2RW = SE1RW+1  ; IP=
	.equ SE3RW = SE2RW+1  ; OP=
	.equ SE4RW = SE3RW+1  ; DB=
	.equ LNRW  = SE4RW+1  ; LINE NUMBER TOKEN
; ENDF
; IF PTDOS
;.equ SE2RW = SE1RW+1  ; XI=
;.equ SE3RW = SE2RW+1  ; FB=
;.equ SE4RW = SE3RW+1  ; CM=
;.equ SE5RW = SE4RW+1  ; CP=
;.equ SE6RW = SE5RW+1  ; OF=
;.equ LNRW  = SE6RW+1  ; LINE NUMBER TOKEN
; ENDF
	.equ EOSRW = LNRW+1  ; THE E-O-S TOKEN
;*
	.equ TOKSW = EOSRW+1  ; ALL < TOKSW ARE SPECIAL TO THE PARSER
;*                              AND ARE NOT FIRST WORDS ON A LINE
;*
	.equ LETRW = TOKSW+0  ;LET
	.equ FORRW = LETRW+1  ; FOR
	.equ PRIRW = FORRW+1  ; PRINT
	.equ NEXTRW = PRIRW+1  ; NEXT
	.equ IFRW = NEXTRW+1  ; IF
	.equ REARW = IFRW+1  ; READ
	.equ IPTRW = REARW+1  ; INPUT
	.equ DATARW = IPTRW+1  ; DATA
	.equ GOTORW = DATARW+1  ; GOTO
	.equ GOSURW = GOTORW+1  ; GOSUB
	.equ RETRW = GOSURW+1  ; RETURN
	.equ DIMRW = RETRW+1  ; DIM
	.equ STORW = DIMRW+1  ; STOP
	.equ ENDRW = STORW+1  ; END
	.equ RESTRW = ENDRW+1  ; RESTORE
	.equ REMRW = RESTRW+1  ; REMARK
	.equ FNERW = REMRW+1  ; FNEND
	.equ DEFRW = FNERW+1  ; DEF
	.equ ONRW = DEFRW+1  ; ON
	.equ OUTRW = ONRW+1  ; OUT
	.equ POKRW = OUTRW+1  ; POKE
	.equ BYERW = POKRW+1  ; BYE
	.equ SETRW = BYERW+1  ; SET
	.equ SCRRW = SETRW+1  ; SCRATCH
	.equ CLRRW = SCRRW+1  ; CLEAR
	.equ XEQRW = CLRRW+1  ; XEQ
	.equ FLERW = XEQRW+1  ; FILE
	.equ REWRW = FLERW+1  ; REWIND
	.equ CLORW = REWRW+1  ; CLOSE
	.equ CURRW = CLORW+1  ; CURSOR
	.equ WATRW = CURRW+1  ; WAIT
	.equ SRCRW  = WATRW+1  ; SEARCH
	.equ TONRW = SRCRW+1  ; TON
	.equ TOFRW = TONRW+1  ; TOFF
	.equ ERSRW = TOFRW+1  ; ERRSET
	.equ ERCRW = ERSRW+1  ; ERRCLR
	.equ FILRW = ERCRW+1  ; FILL
	.equ MATRW = FILRW+1  ; MAT
; IF PTDOS
;.equ PRGRW = MATRW+1  ; PURGE
;.equ LODRW = PRGRW+1  ; LOAD
;.equ PAURW = LODRW+1  ; PAUSE
; ENDF
;.equ  IF 1-PTDOS
.equ PAURW = MATRW+1  ; PAUSE
; ENDF
	.equ EXITRW = PAURW+1  ; EXIT
;*
;*
;*     THE STATEMENT DIVIDER
;*
	.equ TOKST = EXITRW+1  ; ALL < TOKST ARE STATEMENT TOKENS
;*
;*
;*     COMMAND TOKEN DEFINITIONS
;*
	.equ RUNRW = TOKST+0  ;RUN
	.equ LISTRW = RUNRW+1  ; LIST
	.equ CONTRW = LISTRW+1  ; CONTINUE
	.equ EDTRW = CONTRW+1  ; EDIT
	.equ DELRW = EDTRW+1  ; DEL
	.equ GETRW = DELRW+1  ; GET
	.equ APPRW = GETRW+1  ; APPEND
; IF PTDOS
;.equ KILRW = APPRW+1  ; KILL
;.equ CATRW = KILRW+1  ; CAT
;.equ SAVRW = CATRW+1  ; SAVE
; ENDF
; IF 1-PTDOS
.equ SAVRW = APPRW+1  ; SAVE
; ENDF
	.equ RENRW = SAVRW+1  ; RENUMBER
;*
;*
;*     THE COMMAND DIVIDER
;*
	.equ TOKCM = RENRW+1  ; ALL < TOKCM AND >= TOKST ARE COMMANDS
;*
;*
;*     OPERATOR TOKEN DEFINITIONS
;*
	.equ LPARRW = TOKCM+0
	.equ UMINUS = LPARRW+1  ; UNARY MINUS
	.equ ASKRW = UMINUS+1
	.equ PLSRW = ASKRW+1
	.equ MINRW = PLSRW+1
	.equ SLARW = MINRW+1
	.equ ANDRW = SLARW+1
	.equ ORRW = ANDRW+1
	.equ GERW = ORRW+1
	.equ LERW = GERW+1
	.equ NERW = LERW+1
	.equ LTRW = NERW+1
	.equ EQRW = LTRW+1
	.equ GTRW = EQRW+1
	.equ NOTRW = GTRW+1
	.equ XPNRW = NOTRW+1
;*
;*
;*     THE OPERATOR DIVIDER
;*
	.equ TOKOP = XPNRW+1  ; ALL < TOKOP AND >= TOKCM ARE OPERATORS
;*
;*
;*     FUNCTION TOKEN DEFINITIONS
;*
	.equ DMYRW = TOKOP+0  ;DUMMY OF LOW PRESIDENCE FOR EXPR OPERATION
	.equ ABSRW = DMYRW+1  ; ABS
	.equ INTRW = ABSRW+1  ; INT
	.equ LENRW = INTRW+1  ; LEN
	.equ CALLRW = LENRW+1  ; CALL
	.equ RNDRW = CALLRW+1  ; RND
	.equ SGNRW = RNDRW+1  ; SGN
	.equ POSRW = SGNRW+1  ; POS
	.equ EOFRW = POSRW+1  ; EOF
	.equ TYPRW = EOFRW+1  ; TYP
	.equ SINRW = TYPRW+1  ; SIN
	.equ SQRRW = SINRW+1  ; SQR
	.equ FRERW = SQRRW+1  ; FREE
	.equ INPRW = FRERW+1  ; INP
	.equ PEKRW = INPRW+1  ; PEEK
	.equ COSRW = PEKRW+1  ; COS
	.equ EXPRW = COSRW+1  ; EXP
	.equ TANRW = EXPRW+1  ; TANGENT
	.equ ATNRW = TANRW+1  ; ATN
	.equ LG1RW = ATNRW+1  ; LOG10
	.equ LGERW = LG1RW+1  ; LOGE
	.equ SYFRW = LGERW+1  ; SYST
;*
;*
;*     FUNCTION DIVIDER
;*
	.equ TOKFU = SYFRW+1  ;ALL < TOKFU AND >= TOKOP ARE FUNCTIONS
;*
;*
;*     END OF TABLE MARKER
;*
	.equ EOTT = TOKFU+0  ;END OF TOKEN TABLE MARKER


.DSEG 						; Start data segment 
.org 0x100
bgColor:	.byte   2
fgColor:	.byte	2
; term IO overhead
TINFLG: .byte	1			; terminal input flag
TRMIBF:	.byte	32			; terminal input buffer
TRMIFI:	.byte	1			; terminal input fill index
TRMIEI:	.byte	1			; terminal input empty index
;TRMOBF:	.byte	32		; terminal output buffer
;TRMOIX:	.byte	1		; terminal output index
TINCOD:	.byte	1			; terminal input return code


;*
;*
;*
;*     THE  COMMAND  INTERPRETER
;*
;*
;% start of file BSM#CMDI
;% end of file BSM#CMDI
;*
;*
;*     THE  PARSER
;*
;*
;% start of file BSM#PARS
;*  
;*  
BACKF:    .byte 1
;% end of PSM#PARS
;*
;*
;*     THE  INTERPRETER  DRIVER
;*
;*
;% start of BSM#IDVVR
;% end of BSM#IDVR
;*
;*
;*     THE  LINE  EDITOR
;*
;*
;% start of BSM#EDIT
;% end of file BSM#EDIT
;*
;*
;*     THE  COMMANDS
;*
;*
;% start of file BSM#CMDS
;% end of file BSM#CMDS
;*
;*
;*     THE  PROGRAM  STORAGE  COMMANDS
;*
;*
;% start of file BSM#DPSS
;*
;*
;*    RAM STORAGE
;*
XEQFG:    .byte 1             ; XEQ-AFTER-LOAD FLAG
APPFG:    .byte 1             ; APPENDING FLAG
;*
;* 
;% end of file BSM#DPSS
;*
;*
;*     THE  "BASIC"  STATEMENTS
;*
;*
;% start of file BSM#STM1
ERRLN:    .byte 2			; in file BSM#STM1
;% end of file BSM#STM1
;% start of file BSM#STM2
LX:       .byte 1
LY:       .byte 1
;% end of file BSM#STM2
;% start of file BSM#STM3
;% there is no data in  STM3
;% end of file BSM#STM3
;*
;*
;*     THE  FILE  RELATED  STATMENTS
;*
;*
;% start of file BSM#FIL1
;*
;*
;*
;*     EQUATES FOR FILES
;*
;*
;FNSIZ EQU 8+2  FILE NAME LENGTH
;CATT  EQU PFINF+PNAT+PATR
;NATT  EQU PFINF
;BTTXT EQU 86H  TEXT FILES
;BTSCP EQU 85H  SEMI-COMPILED PROGRAM FILES
;BTRND EQU 88H  RANDOM ACCESS FILES
;BTSER EQU 87H  SERIAL ACCESS FILES
;BTMAT EQU 82H  MATRIX FILES
;WPROT EQU PWRI
;RPROT EQU PREA
;DBLKS EQU 0100H
;OFTES EQU 16+1  NUMBER OF OFT ENTRIES (FCB'S) (ONE TAKEN BY...
;*                                COMMANDS)
;TAPPT EQU 0FAH
;*
;*
;*     THE CURRENT FILE CONTROL BLOCK
;*
COFCB:	; EQU $  ADDRESS OF CURRENT OPEN FCB
;*
OFT:	; hard to match existing code
CUFID:    .byte 1             ; USER'S FILE ID
CSFID:    .byte 1             ; SYSTEM'S FILE ID
CACRE:    .byte 1             ; FILE ACCESS RECEIVED
CEOF:     .byte 1             ; FILE STATUS FLAG
CFN:      .byte 6             ; FNSIZ+1  FILE NAME
;CCREB EQU $  ADDR OF CRTEATE BUFFER
CFT:      .byte 1             ; FILE TYPE
CBLKS:    .byte 2             ; BLOCK SIZE (number of bytes to write to tape)
tBlkST:   .byte 2             ; % starting byte of block to write to tape
		  .byte 5
xxFCB:
;*
;OFTEZ EQU $-COFCB  OFT ENTRY SIZE
.equ OFTEZ = xxFCB - COFCB	  ; OFT ENTRY SIZE
;*
;*
;*     THE OPEN FILE TABLE
;*
;OFTZZ EQU OFTES*OFTEZ  THE TOTAL SIZE
;OFT+OFTEZ: ; this ref aligns here
		  .byte 20
          .byte 20
OFTE:     .byte 1			  ; END OF TABLE MARKER
OFCB:     .byte 2             ; CURRENTLY OPEN FCB ADDRESS
;% end of file BSM#FIL2
;*
;*
;*     THE  EXPRESSION  EVALUATOR
;*
;*
;% start of file BSM#EXPR
;% end of file BSM#EXPR
;*
;*
;*     THE  LOGICAL  OPERATORS
;*
;*
;% start of file BSM#LGIC
;% end of file BSM#LGIC
;*
;*
;*     THE  MATH  PACKAGE
;*
;*
;% start of file BSM#MATH
; *

;*
;*
;*     FLOATING  POINT  WORK  SPACE
;*
;
RANLS:    .byte 6              ; last random number seed

FPRAM:  ; EQU $
;*
HOLD1:    .byte 5             ; DIGIT+1
HOLD2:    .byte 5             ; DIGIT+1
HOLD3:    .byte 5             ; DIGIT+1
HOLD4:    .byte 5             ; DIGIT+1
HOLD5:    .byte 5             ; DIGIT+1
HOLD6:    .byte 5             ; DIGIT+1
HOLD7:    .byte 5             ; DIGIT+1
HOLD8:    .byte 5             ; DIGIT+1
          .byte 2             ; 2
BUF:      .byte 4
SIGN:     .byte 1
EXP:      .byte 1
;*   
RCTRL:    .byte 1
RDIGI:    .byte 1
;*
.equ SIGND = HOLD1+DIGIT
.equ EXPD = SIGND+1
.equ XSIGND = EXPD+1  ; HOLDS SIGN WHILE SIGND IS USED FOR ROUNDING.
;*
SINK:     .byte 5             ; FPSIZ-1
FPSINK:   .byte 1
; COPY BSM:FUNS/1  THE EXTENDED FUNCTION STORAGE
;*
;MENT DS 1  MATRIX ENTRY FLAG
;*
XA:       .byte 2
XB:       .byte 2
XC:       .byte 2
XS:       .byte 2
;*
;% end of file  BSM#MATH
;*
;*
;*     THE  FUNCTIONS
;*
;*
;% start of file  BSM#FUN1
;% end of BSM#FUN1
;*
;*
;*     THE  VARRIABLE  AND  CONSTANT  HANDLERS
;*
;*
;% start of BSM#VARC
; *
; *
; *     STRING PROCESSOR PARAMETERS
; *
STRBA: .byte 2				; CURRENT STRING BASE ADDRESS
STRLG: .byte 2				; CURRENT STRING LENGTH
STRMX: .byte 2				; CURRENT STRING MAXIMUM DIMENTION
; *
; *     SYMBOL TABLE STUFF
; *
STA: 	.byte 2				; SYMBOL TABLE FREE POINTER
STNPTR: .byte 2				; ADDR OF POINTER TO LAST SYMTAB ENTRY CREATED
STNBA:	.byte 2				; BASE ADDR OF LAST SYMTAB ENTRY CREATED
;% end of file BSM#VARC
;*
;*
;*     THE  CONVERSION  ROUTINES
;*
;*
;% start of file BSM#CVT1
;*
;*   
;*   
;*   
;*     FORMATTED OUTPUT STORAGE
;*       MUST APPEAR TOGETHER IN THIS ORDER
;* 
TOPT:     .byte 2
V_2618:   .byte 2
COPT:     .byte 1
CFORM:    .byte 1
CWIDTH:   .byte 1
CFRACT:   .byte 1
;*
DOPT:     .byte 1
DFORM:    .byte 1
DWIDTH:   .byte 1
DFRAC:    .byte 1
;*
EFRMF:    .byte 1
;*   
;*     FLOATING INPUT STORAGE
;*   
ESIGN:    .byte 1
CSIGN:    .byte 1
TERMC:    .byte 1
PUTFL:    .byte 1
RESTA:    .byte 2
RESTP:    .byte 2
STRPT:    .byte 2
;*
;*     FLOATING OUTPUT STORAGE
;* 
DPOINT:   .byte 2
EPOINT:   .byte 2
COMCNT:   .byte 1
VALUE:    .byte 2
;% end of file BSM#CVT2
;*
;*
;*     THE  INPUT/OUTPUT  HANDLERS
;*
;*
;ITIMCONST:
;% start of file BSM#IO
;
;
;  I/O RELATED RAM
;
; IF SOLOS
VDMAD:	.byte 2  		; VDM ADDRESS (cursor location)
XOPORT:	.byte 2
XIPORT:	.byte 2
; ENDF
ITIM: 	.byte 2  		; INPUT TIME LIMIT
ICNT: 	.byte 1			; INPUT COUNT LIMIT
LINLEN: .byte 1			; LENGTH OF A LINE
TWIDTH: .byte 1 ;14     ; WIDTH OF A COMMA TAB FIELD
PHEAD:	.byte 1  		; PRINTER HEAD POSITION


;
		.byte 1  		; FOR THE '!'
OBUF:	.byte WMAX+1	; FOR NUMERIC CONVERTIONS
EOBUF:	.byte 1  		; FOR THE '"'
;
IBCNT:	.byte 1			; CURRENT LINE LENGTH (OF INCOMMING LINE)
IBLN: 	.byte 2			; CURRENT LINE NUMBER (OF INCOMMING LINE)
IBUF:	.byte LINMAX/2  ; THE I/O BUFFER
IBUF1:	.byte LINMAX
;
;
; STACK DEFINITION
;
STKTOP: .byte SSIZE  			;THE STACK
.equ CMNDSP = STKTOP+SSIZE-1	; ADDR OF TOP OF STACK
.equ xxY = -CMNDSP+SSIZE-10
.equ CSPM1	= CMNDSP-1
.equ SPCMND	= CMNDSP/256*256
TSTKA: .byte 2  				; TOP OF ARG STACK POINTER
SPTR:  .byte 2					; STACK SAVE FOR 'CONTINUE'
;*
;*
;% end of BSM#IO
;*
;*
;*     THE  ERROR  PROCESSOR
;*
;*
;% start of BSM#ERR
;*
LSTLIN:   .byte 2             ; THE LAST LN ON WHICH THERE WAS AN ERROR
ERRBUF:                       ; EQU $  THE ERROR BUFFER
LSTERR:   .byte 2   ; "  "    ; THE LAST ERROR
          .byte 6   ; " ERROR"
SHORT:    .byte 1   ; " "     ; SPACE FOR LONG, CR FOR SHORT
STOP2:    .byte 8   ; "IN LINE "
ASCLN:    .byte 6   ; "     " ; THE LINE NUMBER IN ASCII + EOM
;*
;*
;% end of file BSM#ERR
;*
;*
;*
;*     FLAGS
;*
ARGF:	.byte 1  ; "LAST WAS ARGUEMENT" FLAG
UNDEF:	.byte 1  ; FLAG FOR ERROR RECOVERY
IFTERM:	.byte 1  ; IF STATEMENT TERMINATOR (EOS OR ELSE)
DIRF:	.byte 1  ; DIRECT FLAG (KEYBOARD MODE STATEMENTS)
CONTF:	.byte 1  ; CAN/CAN'T CONTINUE FLAG
LETFG:	.byte 1  ; THE PRE-PROCESSOR'S 'LET' FLAG
FLNFG:	.byte 1  ; THE UN-PREPROCESSOR'S FIRST LN FLAG
LPHED:	.byte 1  ; INDENTATION COUNT FOR UN-PREPROCESSOR
FORFG:	.byte 1  ; THE "FOR" SEEN FLAG
CTLFG:	.byte 1  ; THE CTRL-CHARACTER EXPANTION FLAG AND CHARACTER
LEDFG:	.byte 1  ; THE LEAD-CHARACTER-SEEN FLAG
INSFG:	.byte 1  ; THE INSERT FLAG
;*
;*     TEXT POINTERS INTO THE BASIC PROGRAM
;*
TXA:	.byte 2
RTXA:	.byte 2
INSA:	.byte 2  ; INSERT ADDRESS FOR EDITOR
FIRST:	.byte 2  ; HOLDS ADDR OF FIRST LINE TO BE AFFECTED
LAST:	.byte 2  ; HOLD ADDR OF END OF LAST LINE TO BE AFFECTED
LHSBA:	.byte 2  ; LEFT  HAND STRING BASE ADDRESS
RHSBA:	.byte 2  ; RIGHT HAND STRING BASE ADDRESS
;*
;*     VARIABLES FOR RENUMBER
;*
BEG:	.byte 2
DEL:	.byte 2
NLN:	.byte 2
;*
;*
;*
;*
;*     THE  MISCELLANEOUS  ROUTINES
;*
;*
;% start if file BSM#MSCS
;% end of BSM#MSCS
;*
;*
;*     THE  USER'S  PROGRAM SPACE
;*
;*
BOFA:	.byte 2 ;XEND+1  BEGINING OF USER'S AREA
EOFA:	.byte 2 ;XEND+1  END      OF USER'S AREA
MEMTOP: .byte 2 ;LAST AVAILABLE WORD TO USER (SET BY INIT)
;*
;*
;*     THE  EXTENDED MATH FUNCTIONS
;*
;*
;XFUNS EQU $
RANC:   .byte 4
;% start of file BSM#FUN2
;*
;*
;*
;*  FLOATING POINT CONSTANTS FOR EXTENDED FUNCTIONS
;*
;% Insert file BSM#CNT1
; IF PX6  IF THIS IS THE 6-DIGIT PRECISION VERSION.
;*
;*
;*  THE 8-DIGIT CONSTANTS
;*
VTEMP:    .byte FPSIZ		; a place for constants to live in SRAM
;%
;...
;*
;*
;% end file insert BSM#CNT1
;*
;*
;% end file BSM#FUN2
;% begin file BSM#FUNS
;*
;*
;*
;*     EXTENDED FUNCTION RAM STORAGE
;*
.equ DIGI1 = DIGIT+1
.equ DIGI2 = DIGI1+1
;*
FTEMP:    .byte DIGI2             ; DIGI2
          .byte 1
FTEM1:    .byte DIGI1
EXPTM:    .byte 1
FTEM2:    .byte DIGI2
SVAL:     .byte 1
SSIGN:    .byte 1
SEXP:     .byte 1
STAKA:    .byte 2
SPNT:     .byte 2
FTMP1:    .byte FPSIZ
;*
;*
;% end file BSM#FUNS
;*
;*
;XMAT  EQU  $
;% begin file BSM#MTC
;*
;%CLA III dumped thes as VARXX
;In keeping with the other code these probably
;are a system of dynamic equates based on FPSIZ
Var3:     .byte 1             ; used in MUL & MIN arg to sub
Var10:    .byte 1             ; used in MUL
Var4:     .byte 2             ; used in mul/min & sbrs
V_3893:   .byte 2             ; used in add sub and inv
TEMP1:    .byte FPSIZ
TEMP2:    .byte FPSIZ
TEMP3:     .byte FPSIZ
.equ MODE	= TEMP3           ; operator mode for mat funs
                              ; overlays the third temp value
V_38A7:   .byte 2
V_38A9:   .byte 2
V_38AB:   .byte 2
V_38AD:   .byte 2
V_38AF:   .byte 2
V_38B1:   .byte 2
V_38B3:   .byte 2
V_38B5:   .byte 2
V_38B7:   .byte 2
V_38B9:   .byte 1
;% end of file BSM#MTC
;% start of file BSM#MTC3
;% begin file BSM#MTC
MA:       .byte 2             ; ADDR OF DEFINITION
MABZ:     .byte 2             ; SIZE IN BYTES
MAROW:    .byte 2             ; ROW SIZE IN ELEMENTS
MACOL:    .byte 2             ; COL SIZE IN ELEMENTS
MADB:     .byte 2             ; DATA BASE ADDRESS
;*
MR:       .byte 2
MRBZ:     .byte 2
MRROW:    .byte 2
MRCOL:    .byte 2
MRDB:     .byte 2
;*
MB:       .byte 2             ; used for scalers
MBBZ:     .byte 2
MBROW:    .byte 2
MBCOL:    .byte 2
MBDB:     .byte 2
;*
;*
;% end of file BSM#MTC3
;*
;*     THE  END
;*
XEND:     .byte 1

.org RAMEND-(40*16)
STACKTOP:	; let stack grow down from display buffer
	.byte 1	; stack writes this location
CC00:		; simulated VDM memory


;******************************************************************************
; 								Interrupt vectors
;******************************************************************************
.cseg
.org 0


	rjmp _main

.org INT0addr				; External Interrupt 0
	reti
	
.org INT1addr				; External Interrupt 1
	reti

.org OC2Aaddr				; Timer/Counter2 Compare Match A
	rjmp ITF0

.org OC2Baddr				; Timer/Counter2 Compare Match B
	reti
.org OVF2addr				; Timer/Counter2 Overflow
	reti
.org ICP1addr				; Timer/Counter Capture Event
	reti
	
.org OC1Aaddr				; Timer/Counter1 Compare Match
	reti
.org OC1Baddr				; Timer/Counter1 Compare Match B
	reti
.org OVF1addr				; Timer/Counter1 Overflow
	reti

.org OC0Aaddr				; TimerCounter0 Compare Match A
	reti 
.org OC0Baddr				; TimerCounter0 Compare Match B
	reti
.org OVF0addr				; Timer/Couner0 Overflow
	reti
	
.org SPIaddr				; Serial Transfer Complete
	reti

.org URXCaddr				; UART, Rx Complete
	rjmp INTRPT
	
.org UDREaddr				; UART Data Register Empty
	reti

.org UTXCaddr				; UART, Tx Complete
	reti

;.org ADCCaddr				; ADC Conversion Complete
;	reti
	
;.org ERDYaddr				; EEPROM Ready
;	reti
	
.org ACIaddr				; Analog Comparator
	reti		

.org INT_VECTORS_SIZE

CONTRL_T:
 .dw IGNORE		; 00 (null) 				CUTER (break)
 .dw TLFT		; 01 ^A (insert character)  CUTER KLEFT
 .dw IGNORE		; 02 ^B
 .dw IGNORE		; 03 ^C (cancel item)
 .dw IGNORE		; 04 ^D (previous item)
 .dw IGNORE		; 05 ^E
 .dw IGNORE		; 06 ^F (next item)
 .dw IGNORE		; 07 ^G
 .dw TLFT		; 08 ^H (cursor back)
 .dw IGNORE		; 09 ^I (command window)
 .dw TLF		; 0A ^J (cursor down)		CUTER KLF
 .dw IGNORE		; 0B ^K (cursor up) (refresh screen)
 .dw TRIT		; 0C ^L (cursor forward)
 .dw TCR		; 0D ^M (carriage return) PCR	
 .dw IGNORE		; 0E ^N
 .dw IGNORE		; 0F ^O
 .dw IGNORE		; 10 ^P
 .dw IGNORE		; 11 ^Q
 .dw IGNORE		; 12 ^R (prior page)
 .dw IGNORE		; 13 ^S (refresh screen)	CUTER KRIGHT
 .dw IGNORE		; 14 ^T (next page)
 .dw IGNORE		; 15 ^U
 .dw IGNORE		; 16 ^V
 .dw IGNORE		; 17 ^W						CUTER KION
 .dw IGNORE		; 18 ^X (delete character)	CUTER KCAN
 .dw IGNORE		; 19 ^Y
 .dw IGNORE		; 1A ^Z						CUTER KIOFF
 .dw TESC		; 1B (escape)				CUTER KESC
 .dw IGNORE		; 1C ^\
 .dw IGNORE		; 1D ^]
 .dw IGNORE		; 1E (home)
 .dw IGNORE		; 1F (control-rubout)


;=====================================================================
;
; memory constants and pre loader strings
;
;====================================================================	


;*
;*
;*
;*     THE  COMMAND  INTERPRETER
;*
;*
;% start of file BSM#CMDI
;*   
;*   
RDYS:     .DB   "READY",0x22
;% end of file BSM#CMDI
;*
;*
;*     THE  PARSER
;*
;*
;% start of file BSM#PARS
;*  
;*
;*
;*      PREPROCESSOR
;*
; ...
;*   
;*   
;*   
;*     SCANNER'S TEXT-TO-TOKEN/TOKEN-TO-TEXT TRANSLATION TABLE
;*   
;  THE RESERVED WORD TABLE
;  
; packed version to conserve flash memory space  AVR asm does not allow misaligned codespace
;
RWT:
.db  0x97, 0x3A, 0x98, 0x4C, 0x45, 0x54, 0x99, 0x46, 0x4F, 0x52, 0xB2, 0x46, 0x49, 0x4C, 0x45, 0x9A
.db  0x50, 0x52, 0x49, 0x4E, 0x54, 0x9B, 0x4E, 0x45, 0x58, 0x54, 0x9C, 0x49, 0x46, 0x9D, 0x52, 0x45
.db  0x41, 0x44, 0xB3, 0x52, 0x45, 0x57, 0x49, 0x4E, 0x44, 0xB4, 0x43, 0x4C, 0x4F, 0x53, 0x45, 0x9E
.db  0x49, 0x4E, 0x50, 0x55, 0x54, 0x9F, 0x44, 0x41, 0x54, 0x41, 0xA0, 0x47, 0x4F, 0x54, 0x4F, 0xA1
.db  0x47, 0x4F, 0x53, 0x55, 0x42, 0xA2, 0x52, 0x45, 0x54, 0x55, 0x52, 0x4E, 0xA3, 0x44, 0x49, 0x4D
.db  0xA4, 0x53, 0x54, 0x4F, 0x50, 0xA5, 0x45, 0x4E, 0x44, 0xA6, 0x52, 0x45, 0x53, 0x54, 0x4F, 0x52
.db  0x45, 0xA7, 0x52, 0x45, 0x4D, 0xA8, 0x46, 0x4E, 0x45, 0x4E, 0x44, 0x84, 0x46, 0x4E, 0xA9, 0x44
.db  0x45, 0x46, 0xAA, 0x4F, 0x4E, 0xAB, 0x4F, 0x55, 0x54, 0xAC, 0x50, 0x4F, 0x4B, 0x45, 0xBE, 0x45
.db  0x58, 0x49, 0x54, 0xB5, 0x43, 0x55, 0x52, 0x53, 0x4F, 0x52, 0xB6, 0x57, 0x41, 0x49, 0x54, 0xB7
.db  0x53, 0x45, 0x41, 0x52, 0x43, 0x48, 0xB8, 0x54, 0x55, 0x4F, 0x4E, 0xB9, 0x54, 0x55, 0x4F, 0x46
.db  0x46, 0xBA, 0x45, 0x52, 0x52, 0x53, 0x45, 0x54, 0xBB, 0x45, 0x52, 0x52, 0x43, 0x4C, 0x52, 0xBC
.db  0x4D, 0x41, 0x54, 0xBD, 0x50, 0x41, 0x55, 0x53, 0x45, 0x8B, 0x5A, 0x45, 0x52, 0x8D, 0x49, 0x44
.db  0x4E, 0x8E, 0x49, 0x4E, 0x56, 0x8F, 0x54, 0x52, 0x4E, 0x80, 0x53, 0x54, 0x45, 0x50, 0x81, 0x54
.db  0x4F, 0x83, 0x54, 0x48, 0x45, 0x4E, 0x85, 0x54, 0x41, 0x42, 0x82, 0x45, 0x4C, 0x53, 0x45, 0x86
.db  0x43, 0x48, 0x52, 0x87, 0x41, 0x53, 0x43, 0x88, 0x45, 0x52, 0x52, 0x89, 0x56, 0x41, 0x4C, 0x8A
.db  0x53, 0x54, 0x52, 0xBF, 0x52, 0x55, 0x4E, 0xB1, 0x58, 0x45, 0x51, 0xC4, 0x47, 0x45, 0x54, 0xC6
.db  0x53, 0x41, 0x56, 0x45, 0xC5, 0x41, 0x50, 0x50, 0x45, 0x4E, 0x44, 0xC0, 0x4C, 0x49, 0x53, 0x54
.db  0xAF, 0x53, 0x43, 0x52, 0xB0, 0x43, 0x4C, 0x45, 0x41, 0x52, 0xC1, 0x43, 0x4F, 0x4E, 0x54, 0x8C
.db  0x43, 0x4F, 0x4E, 0xC7, 0x52, 0x45, 0x4E, 0xAD, 0x42, 0x59, 0x45, 0xC3, 0x44, 0x45, 0x4C, 0xAE
.db  0x53, 0x45, 0x54, 0x92, 0x49, 0x50, 0x3D, 0x93, 0x4F, 0x50, 0x3D, 0x94, 0x44, 0x53, 0x3D, 0x95
.db  0x44, 0x42, 0x3D, 0xC2, 0x45, 0x44, 0x49, 0x54, 0x90, 0x4C, 0x4C, 0x3D, 0x91, 0x4D, 0x4C, 0x3D
.db  0xC8, 0x28, 0xCA, 0x2A, 0xCB, 0x2B, 0xCC, 0x2D, 0xCD, 0x2F, 0xCE, 0x41, 0x4E, 0x44, 0xCF, 0x4F
.db  0x52, 0xD0, 0x3E, 0x3D, 0xD1, 0x3C, 0x3D, 0xD2, 0x3C, 0x3E, 0xD3, 0x3C, 0xD4, 0x3D, 0xD5, 0x3E
.db  0xD6, 0x4E, 0x4F, 0x54, 0xD7, 0x5E, 0xD9, 0x41, 0x42, 0x53, 0xDA, 0x49, 0x4E, 0x54, 0xDB, 0x4C
.db  0x45, 0x4E, 0xDC, 0x43, 0x41, 0x4C, 0x4C, 0xDD, 0x52, 0x4E, 0x44, 0xDE, 0x53, 0x47, 0x4E, 0xDF
.db  0x50, 0x4F, 0x53, 0xE0, 0x45, 0x4F, 0x46, 0xE1, 0x54, 0x59, 0x50, 0xE2, 0x53, 0x49, 0x4E, 0xE3
.db  0x53, 0x51, 0x52, 0xE4, 0x46, 0x52, 0x45, 0x45, 0xE5, 0x49, 0x4E, 0x50, 0xE6, 0x50, 0x45, 0x45
.db  0x4B, 0xE7, 0x43, 0x4F, 0x53, 0xE8, 0x45, 0x58, 0x50, 0xE9, 0x54, 0x41, 0x4E, 0xEA, 0x41, 0x54
.db  0x4E, 0xEB, 0x4C, 0x4F, 0x47, 0x31, 0x30, 0xEC, 0x4C, 0x4F, 0x47, 0xED
endRWT:
.equ EOS = RWT+1 ; EOSRW  END-OF-STATEMENT FOR MULTI-STATEMENTS PER LINE
.equ SZRWT = endRWT - RWT

;*
;*
;*     TOKEN DEFFINITIONS
;*
;*     A TOKEN IS STRUCTURED AS FOLLOWS:
;*  BIT       USE
;*   7     1=TOKEN, 0=NON-TOKEN-BYTE
;*  6-0    TOKEN IDENTIFIER (0-127)
;*
;TOKEN EQU 080H
;*
;*
;*    STATEMENT TOKEN DEFINITIONS
;*
;...
;*
;*
;*     FUNCTION DIVIDER
;*
;TOKFU EQU SYFRW+1  ALL < TOKFU AND >= TOKOP ARE FUNCTIONS
;*
;*
;*     END OF TABLE MARKER
;*
;EOTT EQU TOKFU+0  END OF TOKEN TABLE MARKER
;*
;*
;*     DISPATCH TABLE FOR STATEMENTS AND COMMANDS
;*
DISPT:    .DW   LET           ; LET
          .DW   SFOR          ; SFOR
          .DW   PRINT         ; PRINT
          .DW   NEXT        ; NEXT
          .DW   SIF        ; SIF
          .DW   READ          ; READ
          .DW   INPUT         ; INPUT
          .DW   DATA          ; DATA
          .DW   GOTO        ; GOTO
          .DW   GOSUB        ; GOSUB
          .DW   RETRN         ; RETRN
          .DW   DIM           ; DIM
          .DW   STOP          ; STOP
          .DW   END           ; END
          .DW   RESTOR        ; RESTOR
          .DW   REM           ; REM
          .DW   FN            ; FN
          .DW   DEF           ; DEF
          .DW   ON        ; ON
          .DW   L_OUT         ; OUT
          .DW   FILL          ; FILL
; IF SOLOS
XBYE:     .DW   cRETRN        ; BYE  FOR SOLOS
; ENDF
          .DW   CSET          ; CSET
          .DW   ZAPALL        ; CSCR
          .DW   ZAPER         ; CCLEAR
          .DW   CXEQ          ; CXEQ
          .DW   SFILE         ; 
          .DW   SFREWIND
          .DW   SFCLOSE       ; 
          .DW   SCURS         ; SCURS
          .DW   SWAIT         ; SWAIT
          .DW   SSEAR         ; SSEAR
          .DW   CTON          ; CTON
          .DW   TTOFF         ; CTOFF
          .DW   SERRS        ; SERRS
          .DW   SERRC        ; SERRC
VMAT:     .DW   SMATER        ; SMATER
          .DW   SPAW          ; SPAW
          .DW   EXIT        ; EXIT
;*
;*
;*    COMMANDS
;*
          .DW   CRUN
          .DW   CLIST
          .DW   CCONT
          .DW   CEDIT
          .DW   CDEL
          .DW   CGET
          .DW   CAPP
          .DW   CSAVE
          .DW   CREN
;*
;*
;*      OPERATOR AND FUNCTION DISPATCH AND PRESIDANCE
;*                        TABLES
;*
;OPTAB EQU $
;OPLPAR EQU $
OPTAB:    
	.DB   0x15,low(FLPAR),high(FLPAR),0x11,low(FNEG),high(FNEG)
	.DB   0x0C,low(S_FMUL),high(S_FMUL),0x0A,low(FADD),high(FADD)
 	.DB   0x0A,low(FSUB),high(FSUB),0x0C,low(FDIV),high(FDIV)
 	.DB   0x04,low(AAND),high(AAND),0x02,low(AOR),high(AOR)
 	.DB   0x06,low(AGE),high(AGE),0x06,low(ALE),high(ALE)
 	.DB   0x06,low(ANE),high(ANE),0x06,low(ALT),high(ALT)
 	.DB   0x06,low(AEQ),high(AEQ),0x06,low(AGT),high(AGT)
 	.DB   0x0F,low(ANOT),high(ANOT),0x12,low(FXPN),high(FXPN)


;*
;*
;*     BUILT IN FUNCTION DISPATCH TABLE
;*
OPBOL:    
	.DB   0x01,low(0x0000),high(0x0000),0x15,low(FABS),high(FABS)
	.DB   0x15,low(FINT),high(FINT),0x15,low(0x0000),high(0x0000)
	.DB   0x15,low(0x0000),high(0x0000),0x15,low(FRANDOM),high(FRANDOM)
	.DB   0x15,low(FSGN),high(FSGN),0x15,low(FPOS),high(FPOS)
	.DB   0x15,low(FEOF),high(FEOF),0x15,low(FTYP),high(FTYP)
	.DB   0x15,low(FSIN),high(FSIN),0x15,low(FSQR),high(FSQR)
	.DB   0x15,low(FFREE),high(FFREE),0x15,low(FINP),high(FINP)
	.DB   0x15,low(FEXAM),high(FEXAM),0x15,low(FCOS),high(FCOS)
	.DB   0x15,low(FEXP),high(FEXP),0x15,low(FTANG),high(FTANG)
	.DB   0x15,low(FARCTAN),high(FARCTAN),0x15,low(FLOG10),high(FLOG10)
	.DB   0x15,low(FLOG),high(FLOG),0x00
;*
;*
;% end of PSM#PARS
;*
;*
;*     THE  INTERPRETER  DRIVER
;*
;*
;% start of BSM#IDVVR
;% end of BSM#IDVR
;*
;*
;*     THE  LINE  EDITOR
;*
;*
;% start of BSM#EDIT
;% end of file BSM#EDIT
;*
;*
;*     THE  COMMANDS
;*
;*
;% start of file BSM#CMDS
;*
;*
;*
;*
;*     TABLE OF "SET" ROUTINE ADDRESSES
;*
STBL:     .DW   SETLL         ; SETLL  SET LINE LENGTH
          .DW   SETML         ; SETML  SET MEMORY LIMIT
; IF SOLOS
          .DW   SETIP         ; SETIP
          .DW   SETOP         ; SETOP
          .DW   SETDS         ; SETDS
          .DW   SETDB         ; SETDB
; ENDF
;% end of file BSM#CMDS
;*
;*
;*     THE  PROGRAM  STORAGE  COMMANDS
;*
;*
;% start of file BSM#DPSS
;% end of file BSM#DPSS
;*
;*
;*     THE  "BASIC"  STATEMENTS
;*
;*
;% start of file BSM#STM1
;% end of file BSM#STM1
;% start of file BSM#STM2
STOPS:    .DB   "STOP ",0x22
INSTR:    .DB   "INPUT ERROR, RETYPE ",0x22, 0x00
;% end of file BSM#STM2
;% start of file BSM#STM3
;% there is no data in  STM3
;% end of file BSM#STM3
;*
;*
;*     THE  FILE  RELATED  STATMENTS
;*
;*
;% start of file BSM#FIL1
PtUMsg:   .DB   "PREPARE TAPE UNIT ",0x22," FOR ",0x22, 0x00
RwNDMsg:  .DB   "REWINDING",0x22
TuRdMsg:  .DB   "READING FROM: ",0x22, 0x00
TuWrMsg:  .DB   "WRITING TO: ",0x22, 0x00
;% end of file BSM#FIL2
;*
;*
;*     THE  EXPRESSION  EVALUATOR
;*
;*
;% start of file BSM#EXPR
;% end of file BSM#EXPR
;*
;*
;*     THE  LOGICAL  OPERATORS
;*
;*
;% start of file BSM#LGIC
;% end of file BSM#LGIC
;*
;*
;*     THE  MATH  PACKAGE
;*
;*
;% start of file BSM#MATH
;*   
;*
;start insert of file BSM#RAND  RANDOM NUMBER FUNCTION.
;*
;*  CONSTANTS AND STORAGE FOR RANDOM
;*
;*  RANLS IS THE LAST RANDOM NUMBER GENERATED.
;*  RANOS IS THE ORIGINAL SEED FOR THE RANDOM NUMBER GENERATOR.
;*  RANA AND RANC ARE THE CONSTANTS USED TO GENERATE A
;*  RANDOM NUMBER VIA  X=A*X0+C  X0 IS RANLS.
;*
; IF PX8  8-DIGIT PRECISION
;RANLS:    .DB   0x31, 0x41, 0x59, 0x36, 0x00, 0x81; this may need to be copied to SRAM
;pgm_RANLS:
pgm_RANOS:    .DB   0x31, 0x41, 0x59, 0x33, 0x00, 0x81;  
pgm_RANOSEXP:
.equ RANOS = ((pgm_RANOS+(FPBYT/2))*2)+1

;ENDF
;*
;*
;% end insert of file BSM#RAND
;*
;*
;*
;*
;*     FLOATING  POINT  CONSTANTS
;*
ONE:      
          ; THIS MUCH ONLY FOR 8-DIGIT PRECISION
          .DB   0x10, 0x00, 0x00, 0x00, 0x00, 0x81 
          ; SIGN AND EXPONENT FOR +1.
FPONEEXP:
;*
NONE:     .DB   0x10, 0x00, 0x00, 0x00, 0x01, 0x81
          ;  THE SIGN AND EXPONENT FOR -1.
NONEEXP:
;.equ FPONEEXP = ((NONE-1)*2)+1
.equ FPONE = ((ONE+(FPBYT/2))*2)+1
.equ FPNONE = ((NONE+(FPBYT/2))*2)+1

;% end of file  BSM#MATH
;*
;*
;*     THE  FUNCTIONS
;*
;*
;% start of file  BSM#FUN1
;% end of BSM#FUN1
;*
;*
;*     THE  VARRIABLE  AND  CONSTANT  HANDLERS
;*
;*
;% start of BSM#VARC
;% end of file BSM#VARC
;*
;*
;*     THE  CONVERSION  ROUTINES
;*
;*
;% start of file BSM#CVT1
;% end of file BSM#CVT1
;% start of file BSM#CVT2
;% end of file BSM#CVT2
;*
;*
;*     THE  INPUT/OUTPUT  HANDLERS
;*
;*
;% start of file BSM#IO
;*   
;*   
;*   
;*   
;*   
;*     SPECIAL CHARACTER TABLE
;*   
SCHRT:    
	.db 0x0D,low(PCR),high(PCR), \
		0x7F,low(PBS),high(PBS)
	.db KRIGHT,low(PRIT),high(PRIT), \
		0x16,low(INL0),high(INL0)
	.db 0x08,low(PLFT),high(PLFT), \
		KION,low(PION),high(PION)
	.db KIOFF,low(PIOFF),high(PIOFF), \
		KLF,low(PLF),high(PLF) 
xxSCHRT:
.equ SCHRZ = xxSCHRT-SCHRT/3  ;TABLE LENTH IN ENTRIES
;*
;*
;% end of BSM#IO
;*
;*
;*     THE  ERROR  PROCESSOR
;*
;*
;% start of BSM#ERR
ini_ERMG: .DB   "   ERROR IN LINE      ",0x0D,0 ; THE LINE NUMBER IN ASCII + EOM
;*
;*
;% end of file BSM#ERR
;*
;*
;*     THE  MISCELLANEOUS  ROUTINES
;*
;*
;% start if file BSM#MSCS
;% end of BSM#MSCS
;*
;*
;*     THE  USER'S  PROGRAM SPACE
;*
;*
;*
;*
;*     THE  EXTENDED MATH FUNCTIONS
;*
;*
;XFUNS EQU $
;% start of file BSM#FUN2
;*
;*
;*
;*
;*
;*
;*  FLOATING POINT CONSTANTS FOR EXTENDED FUNCTIONS
;*
;% Insert file BSM#CNT1
; IF PX8  IS THIS THE 8 DIGIT VERSION?
;*
;*
;*  THE 8-DIGIT CONSTANTS
;*
;*
;*
;*
pgm_CN14:     .DB   0x25, 0x00, 0x00, 0x00, 0x00, 0x80; .25 OR 1/4
pgm_CNL10:    .DB   0x43, 0x42, 0x94, 0x48, 0x00, 0x80; LOG10(e)
pgm_CN34:     .DB   0x75, 0x00, 0x00, 0x00, 0x00, 0x80; .75 OR 3/4
pgm_HALF:     .DB   0x50, 0x00, 0x00, 0x00, 0x00, 0x80
pgm_PITWO:    .DB   0x15, 0x70, 0x79, 0x63, 0x00, 0x81; PI/2
pgm_CN23:     .DB   0x23, 0x02, 0x58, 0x51, 0x00, 0x81; LN(10)
pgm_TWOPI:    .DB   0x62, 0x83, 0x18, 0x52, 0x00, 0x81; PI*2
pgm_RANA:     .DB   0x03, 0x14, 0x76, 0x21
pgm_RANC:     .DB   0x02, 0x11, 0x32, 0x48
pgm_A1:       .DB   0x25, 0x00, 0x00, 0x00, 0x00, 0x81
	          .DB   0x80, 0x00, 0x00, 0x00, 0x00, 0x7F
pgm_A2:       .DB   0x11, 0x11, 0x11, 0x11, 0x00, 0x81
	          .DB   0x22, 0x22, 0x22, 0x22, 0x00, 0x80
pgm_A3:       .DB   0x87, 0x50, 0x00, 0x00, 0x00, 0x80
	          .DB   0x27, 0x86, 0x30, 0x00, 0x00, 0x80
pgm_A4:       .DB   0x57, 0x81, 0x25, 0x00, 0x00, 0x80
	          .DB   0x42, 0x18, 0x75, 0x00, 0x00, 0x80
;*
;*  TEN RAISED TO THE .0, .1, .2, ... POWER
;*
pgm_CNTEN:  .DB   0x10, 0x00, 0x00, 0x00, 0x00, 0x81, 0x12, 0x58
			.DB   0x92, 0x54, 0x00, 0x81, 0x15, 0x84, 0x89, 0x32
			.DB   0x00, 0x81, 0x19, 0x95, 0x26, 0x23, 0x00, 0x81
			.DB   0x25, 0x11, 0x88, 0x64, 0x00, 0x81, 0x31, 0x62
			.DB   0x27, 0x77, 0x00, 0x81, 0x39, 0x81, 0x07, 0x17
			.DB   0x00, 0x81, 0x50, 0x11, 0x87, 0x23, 0x00, 0x81
			.DB   0x63, 0x09, 0x57, 0x34, 0x00, 0x81, 0x79, 0x43
			.DB   0x28, 0x23, 0x00, 0x81;      10^.0 = 1
;*
;*
;*  USED BY COSINE TO CALCULATE SINE OR COSINE
;*  POLYNOMIAL COEFFICIENTS
;*
;*
pgm_SINCO:  .DB   0x14, 0x39, 0x41, 0x35, 0x01, 0x82
            .DB   0x42, 0x00, 0x98, 0x06, 0x00, 0x82
			.DB   0x76, 0x70, 0x42, 0x81, 0x01, 0x82
			.DB   0x81, 0x60, 0x52, 0x26, 0x00, 0x82
			.DB   0x41, 0x34, 0x17, 0x02, 0x01, 0x82
			.DB   0x62, 0x83, 0x18, 0x53, 0x00, 0x81,\
			      0xFF, 0x00
;*
;*
;*  FEXP CONSTANTS.  e^X
;*
;*  Ai = [(LN(10))^i]/i! 
;*
pgm_EXPCO:  .DB   0x53, 0x95, 0x51, 0x90, 0x00, 0x80, 0x11, 0x72
			.DB   0x33, 0x54, 0x00, 0x81, 0x20, 0x34, 0x71, 0x12
			.DB   0x00, 0x81, 0x26, 0x50, 0x95, 0x01, 0x00, 0x81
			.DB   0x23, 0x02, 0x58, 0x51, 0x00, 0x81, 0x10, 0x00
			.DB   0x00, 0x00, 0x00, 0x81, 0xFF, 0x00
;*
;*
;*  CONSTANTS FOR ARCTANGENT
;*
;*
;*  POLYNOMIAL COEFFICIENTS
;*
pgm_ATANCO: .DB   0x99, 0x92, 0x33, 0x12, 0x00, 0x7F, 0x14, 0x23
			.DB   0x51, 0x77, 0x01, 0x80, 0x19, 0x99, 0x90, 0x56
			.DB   0x00, 0x80, 0x33, 0x33, 0x33, 0x27, 0x01, 0x80
			.DB   0x10, 0x00, 0x00, 0x00, 0x00, 0x81, 0xFF, 0x00
;*
;*  ATINi = (2*i-1)*PI/14
;*
pgm_ATIN1:  .DB   0x22, 0x43, 0x99, 0x48, 0x00, 0x80
pgm_ATIN2:  .DB   0x67, 0x31, 0x98, 0x42, 0x00, 0x80
pgm_ATIN3:  .DB   0x11, 0x21, 0x99, 0x74, 0x00, 0x81
;*
;*  Bi = TAN(i*PI/7) .   SEE Ai ABOVE
;*
pgm_BKCN:   .DB   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x15
			.DB   0x74, 0x61, 0x00, 0x80, 0x12, 0x53, 0x96, 0x03
			.DB   0x00, 0x81, 0x43, 0x81, 0x28, 0x62, 0x00, 0x81
;*
;*  Ai = i*PI/7
;*
pgm_AKCN:   .DB   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x87
			.DB   0x98, 0x95, 0x00, 0x80, 0x89, 0x75, 0x97, 0x90
			.DB   0x00, 0x80, 0x13, 0x46, 0x39, 0x68, 0x00, 0x81
;*
;*  CONSTANTS FOR NATURAL LOGARITHMS
;*
;*
pgm_LOGCO:  
			.db   00,\
			      0x22, 0x22, 0x22, 0x22, 0x00, 0x80,\
			      0x28, 0x55, 0x55, 0x55, 0x00, 0x80,\
			      0x40, 0x00, 0x00, 0x00, 0x00, 0x80,\
			      0x66, 0x66, 0x66, 0x67, 0x00, 0x80,\
			      0x20, 0x00, 0x00, 0x00, 0x00, 0x81,\
			      0xFF
;*
pgm_LOGN:   .DB   0x50, 0x00, 0x00, 0x00, 0x00, 0x81,\
			 	  0x16, 0x09, 0x43, 0x79, 0x00, 0x81,\
				  0x30, 0x00, 0x00, 0x00, 0x00, 0x81,\
				  0x10, 0x98, 0x61, 0x23, 0x00, 0x81
			.DB   0x20, 0x00, 0x00, 0x00, 0x00, 0x81,\
			      0x69, 0x31, 0x47, 0x18, 0x00, 0x80,\
				  0x20, 0x00, 0x00, 0x00, 0x00, 0x81,\
				  0x69, 0x31, 0x47, 0x18, 0x00, 0x80
;*
pgm_LOGN1:  .DB   0x10, 0x00, 0x00, 0x00, 0x00, 0x81,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
;*
pgm_LOGEA:  .DB   0x69, 0x31, 0x47, 0x18, 0x01, 0x80,\
                  0x51, 0x08, 0x25, 0x62, 0x01, 0x80,\
				  0x35, 0x66, 0x74, 0x94, 0x01, 0x80,\
				  0x22, 0x31, 0x43, 0x55, 0x01, 0x80
			.DB   0x10, 0x53, 0x60, 0x51, 0x01, 0x80
; ENDF
;*
;*
;% end file insert BSM#CNT1
;*
;*
;% end file insert BSM#CNT1
;*
;*
;% end file BSM#FUN2
;*
;*
;XMAT  EQU  $
;% begin file BSM#MTC
;% end file BSM#MTC
;% begin file BSM#ONCE
;%
;% STARTUP code
;%
;*
;*    ONCE ONLY CODE TO PREPARE BASIC FOR PRODUCTION
;*
;*
;*   
;*   
;*  TEXT
;*
;MES1:     .DB   "LAST AVAILABLE MEMORY LOCATION (HEX) IS ",0x22,, 0x00
;MES2:     .DB   "DELETE MATRIX OPERATIONS?  ""
;MES3:     .DB   "DELETE EXTENDED FUNCTIONS?  "", 0x00
;MES4:     .DB   "GIVE FIRST PROTECTED MEMORY LOCATION (HEX):  ",0x22
;                123456789.123456789.123456789.123456789.
MES5:     .DB   "        Processor Technology Corp.",0x22, 0x00
MES6:     .DB   "       Extended BASIC  Revision A",0x22
MES7:     .DB   " COPYRIGHT (C) 1977  ALL RIGHTS RESERVED",0x22,0
MES40:    .DB   " 40th Anniversary edition for ARDUNO UNO",0x22,0
;MES8:     .DB   "SIZING MEMORY",0x22
;MESS9:    .DB   "CHECKSUM FAILED ",0x22, 0x00


; --------------------- interrupt routines -------------------------------

ITF0:
	; delay timer
	push ssreg
	in ssreg,SREG			; must save register as inc affects flags
	subi r16,1     			; bump decrement counter
	sbc r17,zero
	brne ITF0_99
	dec r19
	brne ITF0_99

	; whack the keyboard RTS line
	cbi PORTD,PIND3
kickKBD:
	dec r19
	wdr						; do a signifcant delay
	nop
	brne kickKBD

	sbi PORTD,PIND3			; that shoud give it a good whack on the head

	ldi r19,48				; should be about 5 minutes or so


ITF0_99:
	out SREG,ssreg
	pop ssreg
	RETI

delay:
	; uses timer interrupt pass delay count into R16,r17
	; simple timer polled delay routine
	cp R16,zero
	cpc R17,zero
	brne delay
	ret


delayM:
	; uses timer interrupt pass delay count into R16,r17
	; simple timer polled delay routine
	mov ARGL,R16
	mov ARGH,R17
delay1:
	ldi r16,low(10)
	ldi r17,high(10)

delayX:
	cp R16,zero
	cpc R17,zero
	brne delayX

	subi ARGL,1
	sbc ARGH,zero
	brne delay1

	ret
;********************
;*	INTRPT	    *
;********************
;Serial port interrupt processing
INTRPT:
;	SAVE	A0-A6,D0-D7     	; save all registers
;	LOCK 				; lock out interrupts
	push r24
	in	r24,SREG		; protect status
	push r24
	push ZL
	push ZH				; could implement an input filter here
;;Check auxilliary port
;	MOVB	B.STS,D7		; read status register
;	BTST	#0,D7			; got input character?
;	BNE	IAUXIN
;	BTST	#2,D7			; transmitter empty?
;	BNE	IAUXOT

;Check terminal port

	lds r24,UCSR0A			; read status register
	sbrs r24,RXC0			; got input character?
	rjmp INTRPT_400

;********************
;*	TRMICP	    *
;********************
;Store the character in D1 in the terminal input buffer
;Called by the terminal input interrupt routine
TRMICP:
	lds r24,UDR0
	; can use dedicated registers (3,4,5,8 are available)
	push r24				;save it on stack for later	
	sbrc r24,7				; key down messages have high bit clear
	rjmp TRMICP_10
	inc r5					; downcount can be used for key repeat

; check for ctl-alt-delete
	cpi r24,0x50
	brne TRMICP_AA
	lds r24, TINFLG
	cpi r24,0xEA			; control-alt-ready
	brne TRMICP_AA
	jmp _main				; soft reset

TRMICP_AA:
	lds r24, TINFLG			; check for keyboard ready
	cpi r24,0x06
	brne TRMICP_01			
							; special case caps lock
	ldi ZL,low(layer4*2)
	ldi ZH,high(layer4*2)	

	rjmp TRMICP_03

TRMICP_01:
	andi r24,0x03			; mask the init bits
	cpi r24,0x02			; keyboard is ready
	brne TINFLG_05			; save the keycode (probably from USB backchannel)

	lds r24, TINFLG			; get the common modifier keys
	andi r24,0xC0			; skip over ready bits, the byte now
							; contains the modifier layer
	ldi ZL,low(layer0)
	ldi ZH,high(layer0)	; the key layer matrix is 256 bytes in size
	add ZL,r24				
	adc ZH,zero
	lsl ZL					; multiply by 2 to get bytes
	rol ZH	

TRMICP_03:
	pop r24					; get the matrix address
	add ZL,r24				; z should now point to the keycode
	adc ZH,zero				; modifier keys will have the high bit set
	lpm r24,Z				; will not work on the AT90s8515
	push r24
	sbrs r24,7				; modifier is down, next key probably waiting
	rjmp TINFLG_05			; can log the keystroke
	com r24
	andi r24,0x07			; only the last 2 bits are table addresses
	breq TINFLG_05			; it is an escape code, log it
	; check for caps lock flag
	cpi r24,0x04
	brcs TRMICP_05
							; here we have capslock, ALT or CMD
	brne TRMICP_05A			; we do not have caplock
	push c_tmp
	mov c_tmp,r24
	lds r24,TINFLG
	eor r24,c_tmp
	rjmp TRMICP_05C			; set the caps lock flag

TRMICP_05A:
							; alt & cmnd remain
	swap r24
	lsr r24	

	rjmp TRMICP_05B			; stash them somewhere
		
TRMICP_05:
	swap r24				; save bits high
	lsl r24
	lsl r24					; tables are 64 words

TRMICP_05B:
	push c_tmp				; we need another register
	lds c_tmp,TINFLG
	or r24,c_tmp

TRMICP_05C:
	sts TINFLG,r24		; save the modifier flag for next go round
	pop c_tmp					
	pop r24					;the unused modifier flag
	rjmp INTRPT_400			; next key probably waiting
	
TINFLG_05:
	ldi ZL,low(TRMIBF)		; set buffer base address
	ldi ZH,high(TRMIBF)

	lds r24,TRMIFI			;

	add ZL,r24
	adc ZH,zero

	pop r24
	st Z,r24				; store the input character

	lds r24,TRMIFI
	inc r24					; increment the index
	andi r24,0x1F			;   and make it modulo-32
	sts TRMIFI,r24			; update the input fill index
	rjmp INTRPT_400			; and let the program do the rest

TRMICP_10:
	; the code is init, or a key up message
	cpi r24,0xFA
	brne TRMICP_20
	ldi r24,1
	sts TINFLG,r24
	pop r24
	rjmp INTRPT_400
TRMICP_20:
	cpi r24,0xFD
	brne TRMICP_30
	lds r24,TINFLG
	inc r24
	sts TINFLG,r24
	pop r24
	rjmp INTRPT_400

TRMICP_30:
	;clear of init code
	; is a key up code
	; 2 identical key ups in succession and clear the modifier flag
	dec r5			; down count
	brpl TRMICP_50	
	clr r5			; we had more ups than downs
	lds r24,TINFLG
	andi r24,0x07	; retain the caps lock state
	sts	TINFLG,r24	; clear any modifiers
TRMICP_50:
	pop r24			; no real need to log the key ups

INTRPT_400:

;Restore registers and exit
	pop ZH
	pop ZL
	pop r24
	out SREG,r24
	pop r24
	reti




Serial_println:
	; simple serial hardware printer
	pop ZH		; the return address
	pop ZL

	lsl ZL		; multiply by 2 for lpm instruction
	rol ZH

;Loop here to transfer characters to the output queue buffer
TOTIMD_10:
;	MOVB	(A2)+,D6		; get next character
	lpm
	adiw ZL,1
;	BEQ	20$			;   end
	tst r0
	breq TOTIMD_20

;	TOTCHR
	lds c_tmp,UCSR0A				; send it
l2:	sbrs c_tmp,UDRE0 		; really brain dead delay
	rjmp l2
    sts UDR0,r0
    
;	BR	10$
	rjmp TOTIMD_10

;End of immediate data - update return address on stack
TOTIMD_20: 
;	EVNA	A2			; make it even
	sbrc ZL,0
	adiw ZL,1

;	MOV	A2,6(SP)		;   and update the stack
	lsr ZL				; restore stack pointer for return
	ror ZH

	push ZL				; new return address
	push ZH
	ret					; and off to it

;*******************************************************
;
; main entry point
;
;*******************************************************
;

;INTRPT:
_main:

; this checks how much of the space is held by 
; the memory constants
.equ FixedConst = _main - INT_VECTORS_SIZE



;LET:
;L_0AE3:			; SFOR
;L_0ED1:			; PRINT
;L_0B5E:			; NEXT
;L_0BB7:			; SIF
;L_0E7F:			; READ
;L_105D:			; INPUT
;L_0DA8:			; DATA
;L_0C52:			; GOTO
;L_0CEA:			; GOSUB
;RETRN:			; RETRN
;L_0DD8:			; DIM
;L_0E40:			; STOP
;END:			; END
;L_0EC2:			; RESTOR
;L_0DD3:			; REM
;L_113D:			; FN
;L_1134:			; DEF
;L_0C80:			; ON
;L_1146:			; OUT
;L_1166:			; FILL
;L_11AA:			; SCURS
;L_1173:			; SWAIT
;L_11E9:			; SSEAR
;L_0C70:			; SERRS
;L_0C69:			; SERRC

;L_125E:			; SPAW
;L_0CCB:			; EXIT


;STOP1:
;NEXTS:
;CSERR:

;CRUN:		;BSM#CMDS
;CLIST:		;BSM#CMDS
;CCONT:		;BSM#CMDS
;CEDIT:		;BSM#CMDS
;CDEL:		;BSM#CMDS
;CGET:		;BSM#CMDS  BSM#DPSS
;CAPP:		;BSM#CMDS  BSM#DPSS
;CSAVE:		;BSM#CMDS  BSM#DPSS
;CREN:		;BSM#CMDS  BSM#DPSS



;cFCLOS:	BSM#FIL1
;SFILE: 
;SFREWIND:
;SFCLOSE: 
;TTOFF:			; CTOFF
;TTON:

; IF SOLOS
;cRETRN:			; BYE  FOR SOLOS


;CSET:		BSM#CMDS
;ZAPALL:	BSM#CMDS
;ZAPER:		BSM#CMDS
;CXEQ:		BSM#DPSS
;CTON:		BSM#CMDS
;FLPAR:		BSM#EXPR
;FNEG:		BSM#FUN1
;L_FMUL:	BSM#MATH
;FADD:		BSM#MATH
;FSUB:		BSM#MATH
;FDIV:		BSM#MATH
;AAND:		BSM#LGIC
;AOR:		BSM#LGIC
;AGE:		BSM#LGIC
;ALE:		BSM#LGIC
;ANE:		BSM#LGIC
;ALT:		BSM#LGIC
;AEQ:		BSM#LGIC
;AGT:		BSM#LGIC
;ANOT:		BSM#LGIC
;FABS:		BSM#FUN1
;FINT:		BSM#FUN1
;FRANDOM:	BSM#MATH
;FSGN:		BSM#FUN1
;FPOS:		BSM#FUN1
;FEOF:		BSM#FUN1
;FTYP:		BSM#FUN1
;FSQR:		BSM#FUN1
;FFREE:		BSM#FUN1	
;FINP:		BSM#FUN1
;FEXAM:		BSM#FUN1
;SETLL:		BSM#CMDS
;SETML:		BSM#CMDS
;SETIP:		BSM#CMDS
;SETOP:		BSM#CMDS
;SETDS:		BSM#CMDS
;SETDB:		BSM#CMDS
;PCR:		BSM#IO
;PBS:		BSM#IO
;PRIT:		BSM#IO
;INL0:		BSM#IO
;PLFT:		BSM#IO
;PION:		BSM#IO
;PIOFF:		BSM#IO
;PLF:		BSM#IO


;TLFT:		; 01 ^A (insert character)  CUTER KLEFT
;IGNORE:		; 02 ^B
;TLF:
;TCR:
;TRIT:
;TESC:
;CURLFT:


;CFF:		BSM#MSCS
;DFC:		BSM#MSCS

; ---------------------B A S I C --------
	clr zero		; r14

	sts TINFLG,zero	; keyboard modifiers
	mov r5,zero		; in case of soft reset

	sbi DDRD,DDD3				; this is now the keyboard wakeup line (RTS)
	cbi PORTD,PIND3

	; place the stack high below the note bit state table
	; this is so we can keep memory structures common with emutek
	; for testing.
	ldi    A, high(CMNDSP) 		; SET BASIC'S STACK
	sts SPTR+1,A
	out    SPH, r24
	ldi    A, low(CMNDSP)		
	sts SPTR,A
	out    SPL, r24

	; should be part of init
	sts TRMIFI,zero
	sts TRMIEI,zero
	
	; in the 8080 world these are dynamic set at compile/run time
	ldi ZL,low(XEND)
	ldi ZH,high(XEND)
	sts BOFA,ZL
	sts BOFA+1,ZH
	
	ldi ZL,low(STACKTOP)
	ldi ZH,high(STACKTOP)
	sts MEMTOP,ZL
	sts MEMTOP+1,ZH


;*
;*  INITIALIZE SOME OTHER STUFF
;*   
	lds A,26
	sts CWIDTH,A
	sts CFRACT,zero

	ldi A,14
	sts TWIDTH,A

	call CFF  					; INIT NUMBER PRINTING FORMAT
	call DFC
	ldi A,60					;  INIT LINE LENGTH
	sts LINLEN,A
	

	ser A
	sts OFTE,A

	clr A
	sts INSFG,A
	sts PHEAD,A
	sts CTLFG,A
	STS LEDFG,A

    ldi     A,13          ; CLEAR IF TERM
    sts     IFTERM,A

	; seed the random number generator
	
	ldi XL,low(RANC)
	ldi XH,high(RANC)
	ldi ZL,low(pgm_RANC*2)
	ldi ZH,high(pgm_RANC*2)
	ldi idx,4
blk2:
	lpm
	adiw Z,1
	st X+,r0
	dec idx
	brne blk2
	st X,zero


	call init					; init timers and serial I/O
	WDR



xxxx:	
	call begin
	
	; now refresh the screen with what ever is in memory
	
	mov r12,zero	
	mov r13,zero	; set scroller for 1 to 1
	
	mov r10,zero	; set cursor for top left
	mov r11,zero
		
	ldi YL,low(ERRBUF)
	ldi YH,high(ERRBUF)	; this is the fake pointer address from the VDM used here for 

	ldi ZL,low(ini_ERMG*2)
	ldi ZH,high(ini_ERMG*2)
blk1093:
	lpm
	tst r0
	breq Blkex3
	adiw Z,1
	st Y+,r0
	rjmp blk1093
Blkex3:
	
	
	call refreshScreen  ; for now just refresh what is in memory

;CRLF:		BSM#IO
;pgm_PRNT:	BSM#MSCS


;*
;*  PRINT INTRODUCTION
;*
; PC no jump -5735
IT5:      call    CRLF
; PC no jump -5738
          call    CRLF
;*
          ldi     ZL,low(MES5*2)
          ldi     ZH,high(MES5*2) ; PRINT INTRO
; PC no jump -3797
          call    pgm_PRNT
; PC no jump -5747
          call    CRLF
          ldi     ZL,low(MES6*2)
          ldi     ZH,high(MES6*2)
; PC no jump -3806
          call    pgm_PRNT
; PC no jump -5756
          call    CRLF
          ldi     ZL,low(MES7*2)
          ldi     ZH,high(MES7*2)
; PC no jump -3815
          call    pgm_PRNT
          call    CRLF
          ldi     ZL,low(MES40*2)
          ldi     ZH,high(MES40*2)
; PC no jump -3815
          call    pgm_PRNT
; PC no jump -5765
          call    CRLF
; PC no jump -5768
          call    CRLF
; PC no jump -5771
          call    CRLF
; PC no jump -5774
          call    CRLF
;*
;          ldi     ZL,low(MES8*2)
;          ldi     ZH,high(MES8*2) ; SIZING MEMORY
; PC no jump -3833
;          call    PRNT
; PC no jump -5783
;         call    CRLF


IT4:
	rcall ZAPALL

	sbi PORTD,PORTD3	; wake up the palm keyboard if attached


;*
;*
;*
;*     THE  COMMAND  INTERPRETER
;*
;*
;CCRLF:		BSM#IO
;INLINE:	BSM#IO
;PP:
;LINE: 		BSM#IDVR
;BSERR:
;ISTA1:
;ISTAT:
;GC:		BSM#MSCS


;% start of file BSM#CMDI
;*
;*
;PROGSTART:
;          rjmp    INIT          ; THIS JUMP GETS ZAPPED IN INIT.
;IT:
;          rcall   CRLF
;*
;*
;*
;*   COMMAND PROCESSOR
;*
;EQU 
CMND0:
; PC no jump 9826
          call    CCRLF
          ldi     ZL,low(RDYS*2)
          ldi     ZH,high(RDYS*2) ; READY !!!
; PC no jump 11772
          call    pgm_PRNT
;*
;*
CMND1:    lds     ZL,SPTR     ;LHLD
          lds     ZH,SPTR+1
          out     SPL,ZL        ;SPHL
          out     SPH,ZH        ; SET UP IN CASE OF ERROR IN IMMEDIATE STATEMENT
          ldi     A,1
          sts     DIRF,A        ; 1==>DIRECT MODE
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     BEG,ZL
          sts     BEG+1,ZH
          sts     DEL,ZL
          sts     DEL+1,ZH      ; SET UP SO FINDLN WORKS OK
;*
;* CLOSE ALL FILES
;*
          ldi     ZL,low(OFT+OFTEZ)
          ldi     ZH,high(OFT+OFTEZ)
          ldi     XL,low(OFTEZ)
          ldi     XH,high(OFTEZ)
;*  
FLCLA:    ld      A,Z
          or      A,A           ; test for file control block end
          brmi    CM1           ; no more open FCBs
          breq    L_003B        ; continue closing file control blocks
;*  
          push    ZL
          push    ZH            ; % loop through all file control blocks and close them
          push    XL
          push    XH
          st      Z,XH
          ldi     YL,low(0x0005)
          ldi     YH,high(0x0005)
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          call   cFCLOS
          pop     XH
          pop     XL
          pop     ZH
          pop     ZL
;*   
L_003B:   add     ZL,XL     ; DAD B
          adc     ZH,XH         ; % part of extended cassete package
          rjmp    FLCLA
;*  
CM1:      eor     A,A           ; FLAG SAYING WHETHER ANY EDITS DONE
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
;*  
; PC no jump 9767
CMND2:    call    CCRLF
; PC no jump 9981
          call    INLINE
          rcall   PP
          brcs    CMND3
          rcall   LINE          ; EDIT LINE BECAUSE HAD LINE NUMBER
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; POP PREVIOUS VALUE OF EDIT FLAG
          sec                   ; SET EDIT FLAG TO TRUE
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          rjmp    CMND2
;*  
;*  
CMND3:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; CLEAR OFF EDIT FLAG
          brcc    CMNDX         ; CLEAR VARIABLE AREA IF ANY EDITS WERE DONE
          ldi     ZL,low(CMNDSP)
          ldi     ZH,high(CMNDSP); RESET STACK
          sts     SPTR,ZL
          sts     SPTR+1,ZH
          rcall   ZAPER
CMNDX:    rcall   CMND4
          rjmp    CMND1
;*  
;*  
CMND4:    ldi     ZL,low(IBUF)
          ldi     ZH,high(IBUF) ; SETUP TO EXECUTE THE LINE IN IBUF
          sts     TXA,ZL
          sts     TXA+1,ZH
          ld      A,Z           ; ANYTHING TO EXECUTE?
          cpi     A,13
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; SAVE ANSWER
          brne    PC+3    ; CZ
          call   CRLF          ; NEW LINE
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          brne    PC+2          ; RZ
          ret                   ; ONLY THING IS A CR, ALL DONE
;*  
          subi    A,0x98           ;SUI; TOKSW  TEST FOR STATEMENT/COMMAND
; PC no branch 10735
          brcc    PC+3
          jmp    BSERR         ; ITS GARBAGE
          cpi     A,0x27         ; TOKST-TOKSW
          brcs    CMND5         ; ITS A STATEMENT
          cpi     A,0x30         ; TOKCM-TOKSW
; PC no branch 1314
          brcc    PC+2
          rjmp    ISTA1         ; ITS A COMMAND
; PC no jump 10722
          jmp     BSERR         ; ITS GARBAGE !
;*  
CMND5:    lds     A,IFTERM      ; SAVE IF TERM IN CASE WE WAN'T TO CONTINUE
          clc					; clear the cary edit flag so we can test print vars
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; LATER ON
;*
          rcall   ISTAT         ; 
; PC no jump 11434
          call    GC
          cpi     A,13
          breq    CMND6         ; RESTORE OLD IFTERM
;*
          ldi     ZL,low(IFTERM)
          ldi     ZH,high(IFTERM); 
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 10700
          breq    PC+3
          jmp    BSERR         ; CAN'T CONTINUE AFTER AN ERROR (DON'T NEED TO RESET
;*                                                   THE IF TERM)
CMND6:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET OLD IFTERM
          sts     IFTERM,A
          ret
;*   
;*   
;RDYS:     .DB   "READY""
;*
;*
;% end of file BSM#CMDI
;*
;*
;*     THE  PARSER
;*
;*
;% start of file BSM#PARS
;*
;*
;*      PREPROCESSOR
;*
;*   PREPROCESS LINE IN IBUF BACK INTO IBUF
;*
;* SETS CARRY IF LINE HAS NO LINE NUMBER
;* LEAVES CORRECT LENGTH OF LINE AFTER PREPROCESSING IN IBCN
;* IF THERE IS A LINE NUMBER, IT IS LOCATED AT IBLN=IBUF-2
;*
PP:       ldi     ZL,low(IBUF1)
          ldi     ZH,high(IBUF1); FIRST CHARACTER OF INPUT LINE
          rcall   GF1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; SO GCI WILL WORK
          ldi     A,0xFF        ; INIT THE "LET" FLAG
          sts     LETFG,A
;*
; PC no jump 8371
          call    INTGER        ; SETS CARRY IF NO LINE NUMBER
          sts     IBLN,ZL
          sts     IBLN+1,ZH     ; STORE LINE NUMBER VALUE (EVEN IF NONE)
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; SAVE STATE OF CARRY BIT FOR RETURNING
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; ADDRESS OF NEXT CHARACTER IN IBUF
;*
          ldi     XL,3          ; SET UP INITIAL VALUE FOR COUNT
          ldi     YL,low(IBUF)
          ldi     YH,high(IBUF) ; INITIALIZE WRITE POINTER
;*
;* COME HERE TO CONTINUE PREPROCESSING LINE
;*
PPL:      lds     A,LETFG       ; BUMP LET FLAG
          inc     A
          sts     LETFG,A
          rcall   GF1           ; GET CHAR
          cpi     A,'.'         ; IS IT BY ITSELF?
; PC no branch 78
          brne    PC+2
          rjmp    PPL4A         ; YES--DON'T BOTHER GOING THRU TBL
          push    YL
          push    YH
          ldi     YL,low(RWT*2)
          ldi     YH,high(RWT*2)  ; BASE OF RWT
;*
PPL1:     push    ZL
          push    ZH            ; SAVE TEXT ADDRESS
;          ld      A,Y           ; RW VALUE FOR THIS ENTRY IN RWT
; messy as we need Z to do the lpm
	mov DPL,ZL
	mov DPH,ZH
	mov ZL,YL
	mov ZH,YH
	lpm
	mov A,r0
	mov ZL,DPL
	mov ZH,DPH
          sts     ARGF,A        ; SAVE IN ARGF IN CASE OF MATCH
;*
PPL2:     adiw    YL,1          ; ADVANCE ENTRY POINTER TO NEXT BYTE
          rcall   GF1           ; GET NXT CHAR
          adiw    ZL,1
          mov     XH,A
;         ld      A,Y           ; GET NEXT CHARACTER FROM ENTRY
; messy as we need Z to do the lpm
	mov DPL,ZL
	mov DPH,ZH
	mov ZL,YL
	mov ZH,YH
	lpm
	mov A,r0
	mov ZL,DPL
	mov ZH,DPH
          cp      A,XH          ; COMPARE WITH CHARACTER IN TEXT
          breq    PPL2
;*
;* COME HERE WHEN COMPARISON OF BYTE FAILED
;*
PPL3:     or      A,A
; PC no branch 82
          brpl    PC+2
          rjmp    PPL6          ; JUMP IF FOUND MATCH
          mov     A,XH          ; GET LAST CHAR THAT MISMATCHED
          cpi     A,'.'         ; IS IT ABBREVIATION INDICATOR
          brne    PPL4          ;  NO--ADVANCE SEARCH
          lds     A,ARGF        ; GET IT'S TOKEN
          cpi     A,0xC7        ; TOKCM-1  IS IT AN OPERATOR
; PC no branch 67
          brcc    PC+2
          rjmp    PPL6A         ; NO
          cpi     A,0xD8        ; TOKOP
          brcc    PPL6A         ; NO--THEN IT IS A VALID ABBREVIATION
;*
;* SCAN TO BEGINNING OF NEXT ENTRY
;*
PPL4:     adiw    YL,1          ; ADVANCE ENTRY POINTER
;          ld      A,Y           ; NEXT BYTE IS EITHER CHARACTER OR RW BYTE
; messy as we need Z to do the lpm
	mov DPL,ZL
	mov DPH,ZH
	mov ZL,YL
	mov ZH,YH
	lpm
	mov A,r0
	mov ZL,DPL
	mov ZH,DPH
          or      A,A
          brpl    PPL4          ; KEEP SCANNING IF NOT RW BYTE
;*
;* NOW SEE IF AT END OF TABLE, AND FAIL OR RETURN CONDITION
          pop     ZH
          pop     ZL            ; RESTORE ORIGINAL TXT PTR
          cpi     A,0xED        ; CHECK FOR END OF TABLE BYTE
          brne    PPL1
;*
;* DIDN'T FIND AN ENTRY AT THE GIVEN TEXT ADDR
          pop     YH
          pop     YL
          rcall   GF1           ; GET TEXT CHARACTER
          cpi     A,13          ; CHECK FOR END OF LINE
; PC no branch 185
          brne    PC+2
          rjmp    PPL8          ; GO CLEAN UP AND RETURN
;*
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          lds     A,LETFG       ; TEST LET FLAG
          or      A,A
          brne    PPL3Z
          ldi     A,0x98        ; LETRW
          rcall   PF1
PPL3Z:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
;*   
PPL4A:    rcall   PF1
          adiw    ZL,1          ; ADVANCE TEXT POINTER
          cpi     A,'"'         ; CHECK FOR QUOTED STRING POSSIBILITY
; PC no branch -102
          breq    PC+2
          rjmp    PPL           ; RESTART RWT SEARCH AT NEXT CHARACTER POSITION
          mov     XH,A          ; SET TO STOP SCAN AT CR OR "
;*
;*  PASS A STRING AS IS UNTIL CHAR IN 'B' OR CR
;* 
PPL5:     ld      A,Z           ; NEXT CHARACTER
          cpi     A,13
; PC no branch 155
          brne    PC+2
          rjmp    PPL8
          rcall   PF1
          adiw    ZL,1          ; ADVANCE TEXT POINTER
          cp      A,XH          ; HAVE WE REACHED THE END
; PC no branch -117
          brne    PC+2
          rjmp    PPL           ; BEGIN RWT SCAN FROM NEW CHARACTER POSITION
          rjmp    PPL5
;*
;* FOUND MATCH SO PUT RW VALUE IN TEXT
;*
PPL6A:    adiw    ZL,1          ; PASS THE DOT
PPL6:     pop     r25
;          out     SREG,r25     ; POP PSW  % trashes interrupts
          pop     A             ; REMOVE UNNEEDED TEXT POINTER FROM STACK
          pop     YH
          pop     YL
          lds     A,ARGF        ; GET IT'S TOKEN
          rcall   PF1
          sbiw    ZL,1
          ldi     XH,13
;*
;*  NOW TEST IF COMMAND
;*
          cpi     A,0x97        ; EOSRW
          brne    PPL6B
          ldi     A,0xFF
          sts     LETFG,A
          rjmp    PPL
PPL6B:    cpi     A,0xA7        ; REMRW  IS IT A REMARK
          breq    PPL5          ; YES--PASS THRU AS IS TO A CR
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; GET CARRY (LN FLAG)
          lds     A,ARGF        ; GET TOKEN
          brcc    PPL6C         ; THERE WAS A LN
          cpi     A,0xB1
          breq    PPL5          ; XEQ IN KEYBOARD MODE, COPY AS IS
PPL6C:    cpi     A,0xC4        ; TEST FOR A PS COMMAND
          brcs    PPLN          ; NOT
          cpi     A,0xC7        ; SAVRW+1
; PC no branch -70
          brcc    PC+2
          rjmp    PPL5          ; IS, COPY AS IS
;*   
;*    TEST FOR LINE NUMBERS
;*   
PPLN:     cpi     A,0xA0        ; GOTORW
          breq    PPLN1
          cpi     A,0xA1        ; GOSURW
          breq    PPLN1
          cpi     A,0xBA        ; ERSRW
          breq    PPLN1
          cpi     A,0xBE        ; EXITRW
          breq    PPLN1
          cpi     A,0xA6        ; RESTRW
          breq    PPLN1
          cpi     A,0x83        ; THENRW
          breq    PPLNX
          cpi     A,0x82        ; ELSERW
; PC no branch -208
          breq    PC+2
          rjmp    PPL
;*   
;*   
PPLNX:    ldi     A,0xFF        ; RE INIT LET FLAG
          sts     LETFG,A
;*
PPLN1:    rcall   GF1           ; TO SKIP ANY SPACES
          sbiw    ZL,1          ; FOR SKIIPING OF COMMA
;*
PPLN2:    push    YL
          push    YH
          push    XL
          push    XH            ; SAVE COUNT IN C
          adiw    ZL,1          ; SKIP COMMA
          sts     TXA,ZL
          sts     TXA+1,ZH
; PC no jump 8133
          call    INTGER
          pop     XH
          pop     XL            ; COUNT IN C
          pop     YH
          pop     YL            ; DEST POINTER
          brcc    PPLN3         ; THERE WAS A LINE NUMBER
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          rjmp    PPL
;*
PPLN3:    ldi     A,0x96        ; LNRW
          rcall   PF1
          mov     A,ZL
          rcall   PF1           ; STORE LOW OF LINE NUMBER
          mov     A,ZH
          rcall   PF1           ; STORE HIGH
;*
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          rcall   GF1
          cpi     A,','
; PC no branch -261
          breq    PC+2
          rjmp    PPL
          rjmp    PPLN2
;*   
;*   
;*   COME HERE WHEN DONE
;*   
PPL8:     ldi     A,13
          rcall   PF1
          ldi     ZL,low(IBCNT)
          ldi     ZH,high(IBCNT); SET UP COUNT
          st      Z,XL
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; RESTORE CARRY CONDITION (LINE NUMBER FLAG)
          ret
;*   
;*   
;*   
;*              UN-PREPROCESSOR
;*   
;*  UN PREPROCESS LINE ADDRESSES IN HL TO ADDRESS IN DE
;*  RETURN SOURCE ADDRESS OF CR IN HL ON RETURN
;*  AND LINE LENGTH IN C
;*   
UPPL:     ldi     A,' '
          sts     FLNFG,A       ; SET FIRST LN FLAG
          eor     A,A
          sts     FORFG,A       ; CLEAR "FOR" SEEN FLAG
          sts     BACKF,A       ; CLEAR BACKUP FLAG
          ldi     XL,0          ; DE=BUFFER ADDRESS, C=CHARACTERS IN BUFFER
;*
UPPL9:    adiw    ZL,1          ; SKIP OVER COUNT BYTE OR LNRW TOKEN
          ld      A,Z           ; LOAD LINE NUMBER VALUE
          adiw    ZL,1
          push    ZL
          push    ZH            ; SAVE SOURCE POINTER
          push    XL
          push    XH            ;  SAVE C COUNT
          push    YL
          push    YH            ; SAVE BUFFER POINTER
          ld      ZH,Z          ; REST OF LINE NUMBER
          mov     ZL,A
;*
          lds     A,FLNFG       ; 0==>NO LEADING ANYTHING, ' '==>LEADING BLANKS
; PC no jump 8108
          call    CLNS          ; CONVERT LINE NUMBER
;*
          pop     ZH
          pop     ZL
          push    YL
          push    YH
          _XCHG                 ; OLD BUFFER POINTER IN DE, NEW IN HL
; PC no jump 7940
          _DSUB                 ; FIND DIFFERENCE HL=HL-DE
          pop     YH
          pop     YL            ; NEW BUFFER POINTER
          pop     XH
          pop     XL            ; C HAS CHARACTER COUNT
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; C+L
          mov     XL,ZL         ; INTO C
          pop     ZH
          pop     ZL            ; SOURCE POINTER
;*
          eor     A,A
          sts     FLNFG,A
          lds     A,BACKF       ; UPDATE BACKUP FLAG
          inc     A
          sts     BACKF,A
;*   
UPP0:     adiw    ZL,1
          ld      A,Z           ; NEXT TOKEN IN SOURCE
          or      A,A
          brmi    UPP1          ; JUMP IF TOKEN IS RW
          rcall   PF1           ; PUT CHARACTER IN BUFFER
          cpi     A,13
          brne    UPP0
          ret
;*   
;* COME HERE WHEN RW BYTE DETECTED IN SOURCE
;*   
UPP1:     cpi     A,0x96        ; LNRW  LINE NUMBER?  (IT'S NOT IN THE TABLE)
          breq    UPPL5         ; YES, PROCESS IT
          push    ZL
          push    ZH            ; SAVE SOURCE POINTER
          ldi     ZL,low(RWT*2)
          ldi     ZH,high(RWT*2)  ; BASE OF RWT
UPP2:     
;          ld      c_tmp,Z
          lpm
; note - the adiw messes up the SREG

          adiw    ZL,1          ; ADVANCE RWT POINTER
         cp      A,r0       ; SEE IF RW MATCHED RWT ENTRY
           brne    UPP2          ; CONTINUE LOOKING IF NOT FOUND
;*   
;* FOUND MATCH, ENTRY POINTER LOCATES FIRST CHARACTER
;*  
UPP3:     cpi     A,0x99        ; FORRW  IF "FOR" THEN INC LPHED
          brne    PC+2          ; CZ
          rcall   INCNT
          cpi     A,0x9B        ; IF "NEXT" DEC LPHED
          brne    PC+2          ; CZ
          rcall   DCNT
;*   
          cpi     A,0x84        ; THENRW+1  CK THEN/ELSE/TO/STEP ETC...
          brcs    UPP3D         ; YES, PRINT SPACE
;*   
          cpi     A,0xCE        ; ANDRW  CK FOR AND/OR
          breq    UPP3D
          cpi     A,0xCF        ; ORRW
          breq    UPP3D
;*   
          cpi     A,0x98        ; TOKSW  STATEMENTS HAVE A LEADING SPACE
          brcs    UPP3B
          cpi     A,0xBF        ; TOKST
          brcc    UPP3B
UPP3D:    ldi     A,' '
          rjmp    UPP3C
;*  
UPP3B:    ;ld      A,Z           ; CHARACTER OF RW
          lpm
		  mov A,r0
		  adiw    ZL,1          ; ADVANCE TO NEXT CHAR NOW
          or      A,A           ; SEE IF THIS CAHR IS ASCII TO DISPLAY
          brmi    UPP4
UPP3C:    rcall   PF1
          rjmp    UPP3B         ; KEEP PUTING OUT R/W IN ASCII
;*   		
;*     PROCESS THE DISCOVERY OF A LINE NUMBER
;*   
UPPL5:    lds     A,BACKF       ; GET BACKUP FLAG
          cpi     A,2
; PC no branch -125
          brcc    PC+2
          rjmp    UPPL9         ; CAN'T BACKUP (LESS THAN 3 LN ON THIS LINE YET)
;*   		
          push    ZL
          push    ZH            ; SAVE IT
          sbiw    ZL,1
          sbiw    ZL,1
          sbiw    ZL,1          ; BACK UP TO LAST LNRW (MABY)
          ldi     A,0x96        ; LNRW
          ld      c_tmp,Z
          cp      A,c_tmp
          pop     ZH
          pop     ZL
; PC no branch -136
          breq    PC+2
          rjmp    UPPL9         ; LAST WAS NOT LINE NUMBER, CONTINUE
          ldi     A,','         ; LAST WAS LNRW, SO PRINT A COMMA TO SEPERATE
          rcall   PF1
          rjmp    UPPL9         ; AND PRINT LN
;*   
;*     INDENTATION
;*   
DCNT:     eor     A,A
          sts     FORFG,A       ; FOR SEEN NO LONGER VALID
          lds     A,LPHED
          subi    A,2           ;SUI
          rjmp    INCN1
;*
INCNT:    lds     A,LPHED
          ldi     c_tmp,2        ; ADI
          add     A,c_tmp
          sts     FORFG,A       ; A "FOR" HAS BEEN SEEN
;*
INCN1:    brcc    INCN2         ; NO FURTHER BACK THAN 0
          eor     A,A
INCN2:    cpi     A,27          ; 32-6+1
          brcs    PC+2          ; RNC
          ret                   ; NO FURTHER THEN 1/2 SCREEN
          sts     LPHED,A
          ret
;*   
;*  COME HERE IF DONE WITH RW TRANSFER
;*   
UPP4:     pop     ZH
          pop     ZL            ; SOURCE POINTER
          ld      A,Z           ; GET SOURCE CHAR AGAIN
;*   
          cpi     A,0xCE        ; ANDRW
          breq    SPOUT
          cpi     A,0xCF        ; ORRW
          breq    SPOUT
          cpi     A,0xD6        ; NOTRW
          breq    SPOUT
;*   
          cpi     A,0x84        ; THENRW+1
          brcs    SPOUT
;*   
          cpi     A,0x98        ; TOKSW
; PC no branch -166
          brcc    PC+2
          rjmp    UPP0
          cpi     A,0xBF        ; TOKST
; PC no branch -171
          brcs    PC+2
          rjmp    UPP0
;*   
          cpi     A,0xA7        ; REMRW  TEST FOR REM
; PC no branch -176
          brne    PC+2
          rjmp    UPP0
          cpi     A,0x84        ; FNRW  AND FUNCTIONS (USER)
; PC no branch -181
          brne    PC+2
          rjmp    UPP0
;*
SPOUT:    ldi     A,' '         ; OUTPUT A SPACE
          rcall   PF1
          rjmp    UPP0
;*   
;*   
;*     "GET" FILTER #1 FOR PARSER
;*   
GF1N:     adiw    ZL,1          ; SKIP BLANKS
;*
GF1:      ld      A,Z           ; GET THE CHARACTER
          cpi     A,13
          brne    PC+2          ; RZ
          ret                   ; NEVER FOOL WITH THOSE
;*
          cpi     A,'!'         ; SPACE OR CTRL-CHAR?
          brcs    GF1N          ; GET NEXT CHARACTER
          cpi     A,'a'         ; LOWER CASE A
          brcc    PC+2          ; RC
          ret
          cpi     A,'{'         ; LOWER CASE Z + 1
          brcs    PC+2          ; RNC
          ret
          andi    A,0xDF        ; UPSHIFT
          ret
;*   
;*   
;*     "PUT" FILTER #1 FOR PARSER
;*   
PF1:      push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          ldi     A,LINMAX     ; 0x84
          cp      A,XL
; PC no branch 10050
          brcc    PC+3
          jmp    LLERR         ; LINE TOO LONG
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
;*   
          st      Y,A
          adiw    YL,1
          inc     XL
          ret
;*  
;*  
;BACKF:    .byte 1
;*   
;*   
;*   
;*     SCANNER'S TEXT-TO-TOKEN/TOKEN-TO-TEXT TRANSLATION TABLE
;*   
;RWT EQU $  THE RESERVED WORD TABLE
;*  
;RWT:      .DB   0x97, 0x00    ; EOSRW  END-OF-STATEMENT FOR MULTI-STATEMENTS PER LINE
;EOS:      .DB   ":", 0x00
;*
;          .DB   0x98, 0x00    ; LETRW  LET
;          .DB   "LET", 0x00
;*
;          .DB   0x99, 0x00    ; FORRW  FOR
;          .DB   "FOR", 0x00
;*
;          .DB   0xB2, 0x00    ; FLERW  FILE
;          .DB   "FILE"
;*
;          .DB   0x9A, 0x00    ; PRIRW  PRINT
;          .DB   "PRINT", 0x00
;*
;          .DB   0x9B, 0x00    ; NEXTRW  NEXT
;          .DB   "NEXT"
;*
;          .DB   0x9C, 0x00    ; IFRW  IF
;          .DB   "IF"
;*
;          .DB   0x9D, 0x00    ; REARW  READ
;          .DB   "READ"
;*
;          .DB   0xB3, 0x00    ; REWRW  REWIND
;          .DB   "REWIND"
;*
;          .DB   0xB4, 0x00    ; CLORW  CLOSE
;          .DB   "CLOSE", 0x00
;*
;          .DB   0x9E, 0x00    ; IPTRW  INPUT
;          .DB   "INPUT", 0x00
;*
;          .DB   0x9F, 0x00    ; DATARW  DATA
;          .DB   "DATA"
;*
;          .DB   0xA0, 0x00    ; GOTORW  GOTO
;          .DB   "GOTO"
;*
;          .DB   0xA1, 0x00    ; GOSURW  GOSUB
;          .DB   "GOSUB", 0x00
;*
;          .DB   0xA2, 0x00    ; RETRW  RETURN
;          .DB   "RETURN"
;*
;          .DB   0xA3, 0x00    ; DIMRW  DIM
;          .DB   "DIM", 0x00
;*
;          .DB   0xA4, 0x00    ; STORW  STOP
;          .DB   "STOP"
;*
;          .DB   0xA5, 0x00    ; ENDRW  END
;          .DB   "END", 0x00
;*
;          .DB   0xA6, 0x00    ; RESTRW  RESTORE
;          .DB   "RESTORE", 0x00
;*
;          .DB   0xA7, 0x00    ; REMRW  REM
;          .DB   "REM", 0x00
;*
;          .DB   0xA8, 0x00    ; FNERW  FNEND
;          .DB   "FNEND", 0x00
;*
;          .DB   0x84, 0x00    ; FNRW  FN
;          .DB   "FN"
;*
;          .DB   0xA9, 0x00    ; DEFRW  DEF
;          .DB   "DEF", 0x00
;*
;          .DB   0xAA, 0x00    ; ONRW  ON
;          .DB   "ON"
;*
;          .DB   0xAB, 0x00    ; OUTRW  OUT
;          .DB   "OUT", 0x00
;*
;          .DB   0xAC, 0x00    ; POKRW  FILL
;          .DB   "POKE"
;*
;          .DB   0xBE, 0x00    ; EXITRW  EXIT
;          .DB   "EXIT"
;*
;          .DB   0xB5, 0x00    ; CURRW  CURSOR
;          .DB   "CURSOR"
;*
;          .DB   0xB6, 0x00    ; WATRW  WAIT
;          .DB   "WAIT"
;*
;          .DB   0xB7, 0x00    ; SRCRW  SEARCH
;          .DB   "SEARCH"
;*
;          .DB   0xB8, 0x00    ; TONRW  TON
;          .DB   "TUON"
;*
;          .DB   0xB9, 0x00    ; TOFRW  TOFF
;          .DB   "TUOFF", 0x00
;*
;          .DB   0xBA, 0x00    ; ERSRW  ERRSET
;          .DB   "ERRSET"
;*
;          .DB   0xBB, 0x00    ; ERCRW  ERRCLR
;          .DB   "ERRCLR"
;*
;          .DB   0xBC, 0x00    ; MATRW  MAT
;          .DB   "MAT", 0x00
;*
;          .DB   0xBD, 0x00    ; PAURW  PAUSE
;          .DB   "PAUSE", 0x00
;*
;          .DB   0x8B, 0x00    ; ZERRW  MAT ZER
;          .DB   "ZER", 0x00
;*
;          .DB   0x8D, 0x00    ; IDNRW  MAT IDN
;          .DB   "IDN", 0x00
;*
;          .DB   0x8E, 0x00    ; INVRW  MAT INV
;          .DB   "INV", 0x00
;*
;          .DB   0x8F, 0x00    ; TRNRW  MAT TRN
;          .DB   "TRN", 0x00
;*
;          .DB   0x80, 0x00    ; STEPRW  STEP
;          .DB   "STEP"
;*
;          .DB   0x81, 0x00    ; TORW  TO
;          .DB   "TO"
;*
;          .DB   0x83, 0x00    ; THENRW  THEN
;          .DB   "THEN"
;*
;          .DB   0x85, 0x00    ; TABRW  TAB
;          .DB   "TAB", 0x00
;*
;          .DB   0x82, 0x00    ; ELSERW  ELSE
;          .DB   "ELSE"
;*
;          .DB   0x86, 0x00    ; CHRRW  CHR
;          .DB   "CHR", 0x00
;*
;          .DB   0x87, 0x00    ; ASCRW  ASC
;          .DB   "ASC", 0x00
;*
;          .DB   0x88, 0x00    ; ERRRW  ERR
;          .DB   "ERR", 0x00
;*
;          .DB   0x89, 0x00    ; VALRW  VAL
;          .DB   "VAL", 0x00
;*
;          .DB   0x8A, 0x00    ; STRRW  STR
;          .DB   "STR", 0x00
;*
;          .DB   0xBF, 0x00    ; RUNRW  RUN
;          .DB   "RUN", 0x00
;*
;          .DB   0xB1, 0x00    ; XEQRW  XEQ
;          .DB   "XEQ", 0x00
;*
;          .DB   0xC4, 0x00    ; GETRW  GET
;          .DB   "GET", 0x00
;*
;          .DB   0xC6, 0x00    ; SAVRW  SAVE
;          .DB   "SAVE"
;*
;          .DB   0xC5, 0x00    ; APPRW  APPEND
;          .DB   "APPEND"
;*
;          .DB   0xC0, 0x00    ; LISTRW  LIST
;          .DB   "LIST"
;*
;          .DB   0xAF, 0x00    ; SCRRW  SCR
;          .DB   "SCR", 0x00
;*
;          .DB   0xB0, 0x00    ; CLRRW  CLEAR
;          .DB   "CLEAR", 0x00
;*
;          .DB   0xC1, 0x00    ; CONTRW  CONT
;          .DB   "CONT"
;*
;          .DB   0x8C, 0x00    ; CONRW  CON (MAT)
;          .DB   "CON", 0x00
;*
;          .DB   0xC7, 0x00    ; RENRW  REN
;          .DB   "REN", 0x00
;*
;          .DB   0xAD, 0x00    ; BYERW  BYE
;          .DB   "BYE", 0x00
;*
;          .DB   0xC3, 0x00    ; DELRW  DEL
;          .DB   "DEL", 0x00
;*
;          .DB   0xAE, 0x00    ; SETRW  SET
;          .DB   "SET", 0x00
;*
;          .DB   0x92, 0x00    ; SE2RW  IP=
;          .DB   "IP=", 0x00
;*
;          .DB   0x93, 0x00    ; SE3RW  OP=
;          .DB   "OP=", 0x00
;*
;          .DB   0x94, 0x00    ; SE1RW  DS=
;          .DB   "DS=", 0x00
;*
;          .DB   0x95, 0x00    ; SE4RW  DB=
;          .DB   "DB=", 0x00
;*
;          .DB   0xC2, 0x00    ; EDTRW  EDIT
;          .DB   "EDIT"
;*
;          .DB   0x90, 0x00    ; SELRW  LL=
;          .DB   "LL=", 0x00
;*
;          .DB   0x91, 0x00    ; SEMRW  ML=
;          .DB   "ML=", 0x00
;*
;          .DB   0xC8, 0x00    ; LPARRW  OPEN PARENTHESIS
;          .DB   "(", 0x00
;*
;          .DB   0xCA, 0x00    ; ASKRW  MULTIPLY
;          .DB   "*", 0x00
;*
;          .DB   0xCB, 0x00    ; PLSRW  ADDITION
;          .DB   "+", 0x00
;*
;          .DB   0xCC, 0x00    ; MINRW  SUBTRACTION
;          .DB   "-", 0x00
;*
;          .DB   0xCD, 0x00    ; SLARW  DIVISION
;          .DB   "/", 0x00
;*
;          .DB   0xCE, 0x00    ; ANDRW  LOGICAL AND
;          .DB   "AND", 0x00
;*
;          .DB   0xCF, 0x00    ; ORRW  LOGICAL OR
;          .DB   "OR"
;*
;          .DB   0xD0, 0x00    ; GERW  GREATER THAN OR EQUAL TO
;          .DB   ">="
;*
;          .DB   0xD1, 0x00    ; LERW  LESS THAN OR EQUAL TO
;          .DB   "<="
;*
;          .DB   0xD2, 0x00    ; NERW  NOT EQUAL TO
;          .DB   "<>"
;*
;          .DB   0xD3, 0x00    ; LTRW  LESS THAN
;          .DB   "<", 0x00
;*
;          .DB   0xD4, 0x00    ; EQRW  EQUAL TO
;          .DB   "=", 0x00
;*
;          .DB   0xD5, 0x00    ; GTRW  GREATER THAN
;          .DB   ">", 0x00
;*
;          .DB   0xD6, 0x00    ; NOTRW  LOGICAL NOT
;          .DB   "NOT", 0x00
;*
;          .DB   0xD7, 0x00    ; XPNRW  EXPONENTIATION
;          .DB   "^", 0x00
;*
;          .DB   0xD9, 0x00    ; ABSRW  ABS
;          .DB   "ABS", 0x00
;*
;          .DB   0xDA, 0x00    ; INTRW  INT
;          .DB   "INT", 0x00
;*
;          .DB   0xDB, 0x00    ; LENRW  LEN
;          .DB   "LEN", 0x00
;*
;          .DB   0xDC, 0x00    ; CALLRW  CALL
;          .DB   "CALL"
;*
;          .DB   0xDD, 0x00    ; RNDRW  RND
;          .DB   "RND", 0x00
;*
;          .DB   0xDE, 0x00    ; SGNRW  SGN
;          .DB   "SGN", 0x00
;*
;          .DB   0xDF, 0x00    ; POSRW  POS
;          .DB   "POS", 0x00
;*
;          .DB   0xE0, 0x00    ; EOFRW  EOF
;          .DB   "EOF", 0x00
;*
;          .DB   0xE1, 0x00    ; TYPRW  TYP
;          .DB   "TYP", 0x00
;*
;          .DB   0xE2, 0x00    ; SINRW  SIN
;          .DB   "SIN", 0x00
;*
;          .DB   0xE3, 0x00    ; SQRRW  SQR
;          .DB   "SQR", 0x00
;*
;          .DB   0xE4, 0x00    ; FRERW  FREE
;          .DB   "FREE"
;*
;          .DB   0xE5, 0x00    ; INPRW  INP
;          .DB   "INP", 0x00
;*
;          .DB   0xE6, 0x00    ; PEKRW  EXAM
;          .DB   "PEEK"
;*
;          .DB   0xE7, 0x00    ; COSRW  COS
;          .DB   "COS", 0x00
;*
;          .DB   0xE8, 0x00    ; EXPRW  EXP
;          .DB   "EXP", 0x00
;*
;          .DB   0xE9, 0x00    ; TANRW  TAN
;          .DB   "TAN", 0x00
;*
;          .DB   0xEA, 0x00    ; ATNRW  ATN
;          .DB   "ATN", 0x00
;*
;          .DB   0xEB, 0x00    ; LG1RW  LOG (BASE 10)
;          .DB   "LOG10", 0x00
;*
;          .DB   0xEC, 0x00    ; LGERW  LOG (BASE e)
;          .DB   "LOG", 0x00
;          .DB   0xED, 0x00    ; EOTT  END OF TABLE
;*
;*
;*     TOKEN DEFFINITIONS
;*
;*     A TOKEN IS STRUCTURED AS FOLLOWS:
;*  BIT       USE
;*   7     1=TOKEN, 0=NON-TOKEN-BYTE
;*  6-0    TOKEN IDENTIFIER (0-127)
;*
;.equ TOKEN = 0x80
;*
;*
;*    STATEMENT TOKEN DEFINITIONS
;*
;.equ STEPRW = TOKEN+0  ;STEP
;.equ TORW = STEPRW+1  ; TO
;.equ ELSERW = TORW+1  ; ELSE
;.equ THENRW = ELSERW+1  ; THEN
;.equ FNRW = THENRW+1  ; FN
;.equ TABRW = FNRW+1  ; TAB
;.equ CHRRW = TABRW+1  ; CHR
;.equ ASCRW = CHRRW+1  ; ASC
;.equ ERRRW = ASCRW+1  ; ERR
;.equ VALRW = ERRRW+1  ; VAL
;.equ STRRW = VALRW+1  ; STR
;.equ ZERRW = STRRW+1  ; MAT ZER
;.equ CONRW = ZERRW+1  ; MAT CON
;.equ IDNRW = CONRW+1  ; MAT IDN
;.equ INVRW = IDNRW+1  ; MAT INV
;.equ TRNRW = INVRW+1  ; MAT TRN
;.equ SELRW = TRNRW+1  ; LL=
;.equ SEMRW = SELRW+1  ; ML=
;.equ SE1RW = SEMRW+1  ; DS=
; IF SOLOS
;.equ SE2RW = SE1RW+1  ; IP=
;.equ SE3RW = SE2RW+1  ; OP=
;.equ SE4RW = SE3RW+1  ; DB=
;.equ LNRW  = SE4RW+1  ; LINE NUMBER TOKEN
; ENDF
; IF PTDOS
;.equ SE2RW = SE1RW+1  ; XI=
;.equ SE3RW = SE2RW+1  ; FB=
;.equ SE4RW = SE3RW+1  ; CM=
;.equ SE5RW = SE4RW+1  ; CP=
;.equ SE6RW = SE5RW+1  ; OF=
;.equ LNRW  = SE6RW+1  ; LINE NUMBER TOKEN
; ENDF
;.equ EOSRW = LNRW+1  ; THE E-O-S TOKEN
;*
;.equ TOKSW = EOSRW+1  ; ALL < TOKSW ARE SPECIAL TO THE PARSER
;*                              AND ARE NOT FIRST WORDS ON A LINE
;*
;.equ LETRW = TOKSW+0  ;LET
;.equ FORRW = LETRW+1  ; FOR
;.equ PRIRW = FORRW+1  ; PRINT
;.equ NEXTRW = PRIRW+1  ; NEXT
;.equ IFRW = NEXTRW+1  ; IF
;.equ REARW = IFRW+1  ; READ
;.equ IPTRW = REARW+1  ; INPUT
;.equ DATARW = IPTRW+1  ; DATA
;.equ GOTORW = DATARW+1  ; GOTO
;.equ GOSURW = GOTORW+1  ; GOSUB
;.equ RETRW = GOSURW+1  ; RETURN
;.equ DIMRW = RETRW+1  ; DIM
;.equ STORW = DIMRW+1  ; STOP
;.equ ENDRW = STORW+1  ; END
;.equ RESTRW = ENDRW+1  ; RESTORE
;.equ REMRW = RESTRW+1  ; REMARK
;.equ FNERW = REMRW+1  ; FNEND
;.equ DEFRW = FNERW+1  ; DEF
;.equ ONRW = DEFRW+1  ; ON
;.equ OUTRW = ONRW+1  ; OUT
;.equ POKRW = OUTRW+1  ; POKE
;.equ BYERW = POKRW+1  ; BYE
;.equ SETRW = BYERW+1  ; SET
;.equ SCRRW = SETRW+1  ; SCRATCH
;.equ CLRRW = SCRRW+1  ; CLEAR
;.equ XEQRW = CLRRW+1  ; XEQ
;.equ FLERW = XEQRW+1  ; FILE
;.equ REWRW = FLERW+1  ; REWIND
;.equ CLORW = REWRW+1  ; CLOSE
;.equ CURRW = CLORW+1  ; CURSOR
;.equ WATRW = CURRW+1  ; WAIT
;.equ SRCRW  = WATRW+1  ; SEARCH
;.equ TONRW = SRCRW+1  ; TON
;.equ TOFRW = TONRW+1  ; TOFF
;.equ ERSRW = TOFRW+1  ; ERRSET
;.equ ERCRW = ERSRW+1  ; ERRCLR
;.equ FILRW = ERCRW+1  ; FILL
;.equ MATRW = FILRW+1  ; MAT
; IF PTDOS
;.equ PRGRW = MATRW+1  ; PURGE
;.equ LODRW = PRGRW+1  ; LOAD
;.equ PAURW = LODRW+1  ; PAUSE
; ENDF
;.equ  IF 1-PTDOS
;.equ PAURW = MATRW+1  ; PAUSE
; ENDF
;.equ EXITRW = PAURW+1  ; EXIT
;*
;*
;*     THE STATEMENT DIVIDER
;*
;.equ TOKST = EXITRW+1  ; ALL < TOKST ARE STATEMENT TOKENS
;*
;*
;*     COMMAND TOKEN DEFINITIONS
;*
;.equ RUNRW = TOKST+0  ;RUN
;.equ LISTRW = RUNRW+1  ; LIST
;.equ CONTRW = LISTRW+1  ; CONTINUE
;.equ EDTRW = CONTRW+1  ; EDIT
;.equ DELRW = EDTRW+1  ; DEL
;.equ GETRW = DELRW+1  ; GET
;.equ APPRW = GETRW+1  ; APPEND
; IF PTDOS
;.equ KILRW = APPRW+1  ; KILL
;.equ CATRW = KILRW+1  ; CAT
;.equ SAVRW = CATRW+1  ; SAVE
; ENDF
; IF 1-PTDOS
;.equ SAVRW = APPRW+1  ; SAVE
; ENDF
;.equ RENRW = SAVRW+1  ; RENUMBER
;*
;*
;*
;% end of BSM#PARS
;*
;*
;*     THE  INTERPRETER  DRIVER
;*
;*
;% start of BSM#IDVVR
;*
;*
;*   INTERPRETER DRIVER
;*
; IF SOLOS
;XSYT5 EQU $
; ENDF
;*
; PC no jump -17769
ILOOP:    call    cSINP         ; CHECK FOR CHARACTER INPUT
          breq    PC+3          ; CNZ
          call    PCHK1         ; IF ONE WAS
          call    ISTAT         ; INTERPRET STATEMENT
; PC no jump 9804
IL1:      call    JOE           ; TEST FOR JUNK ON END AND MOVE TO NEXT STATEMENT
          brcc    ILOOP         ; CONTINUE IF NOT AT END OF PROGRAM
; PC no jump 2270
          jmp     END           ; EXECUTE END STATEMENT
;*
;*
;*    INTERPRET STATEMENT LOCATED BY TXA
;*
ISTAT:    eor     A,A
          sts     UNDEF,A       ; THERE IS NOTHING TO UNDIFINE (YET)
;*   
ISTA0:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; GET NEXT RW
          ld      A,Z
          subi    A,0x98           ;SUI; TOKSW  CHECK THAT IT IS NOT A 'SECOND WORD' TOKEN
; PC no branch 9416
          brcc    PC+3
          jmp    BSERR         ; ALSO CATCHES NON-RW'S
          cpi     A,'''         ; TOKST-TOKSW
; PC no branch 9411
          brcs    PC+3
          jmp    BSERR
;*   
ISTA1:    adiw    ZL,1          ; ADVANCE TEXT POINTER PAST RW
          sts     TXA,ZL
          sts     TXA+1,ZH
;*   
	lsl A			; multiply by 2 for offset
	;DJMP	D1			; directed jump on function code
	ldi ZL,low(DISPT*2)
	ldi ZH,high(DISPT*2)
	add ZL,A
	adc ZH,zero
	lpm 
	mov c_tmp,r0
	adiw Z,1
	lpm
	mov ZH,r0
	mov ZL,c_tmp
;*   
          ijmp         ; PCHL   ; BRANCH TO STATEMENT OR COMMAND
;*
;*
;% end of BSM#IDVR
;*
;*
;*     THE  LINE  EDITOR
;*
;*
;% start of BSM#EDIT
;*
;*
;*
;*      LINE EDITOR
;*
;*   FIRST DECIDE IF APPEND OR INSERT
;*
LINE:     lds     ZL,IBLN     ;LHLD
          lds     ZH,IBLN+1     ; LINE NUMBER OF EDITING COMMAND
;#if _code_Revision_
; MOV A,H
; ORA L
; JZ LNERR  CANT BE LINE 0
; INX H
; MOV A,H
; ORA L
; DCX H
; JZ LNERR  CANT BE 65535
;#endif
          _XCHG
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ldi     XL,low(0x0000)
          ldi     XH,high(0x0000)
;*   
LIN0:     ld      A,Z
          mov     XL,A          ; LINE COUNT TO C
          dec     A
          breq    APP
          adiw    ZL,1
; PC no jump 9564
          call    DCMP
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
          breq    INSR
          brcs    INSR
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          rjmp    LIN0
;*
;*    APPEND LINE AT END CASE
;*
APP:      lds     A,IBCNT       ; DON'T APPEND NULL LINE
          cpi     A,4
          brne    PC+2          ; RZ
          ret
          rcall   FULL
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1     ; PLACE LINE IN FILE
          rcall   IMOV
          ldi     c_tmp,1
          st      Z,c_tmp
          sts     EOFA,ZL
          sts     EOFA+1,ZH
          ret
;*
;*THE STATEMENT LINE NUMBER WAS NOT GREATER THAN THE LAST LI
;*
INSR:     ld      XH,Z          ; OLD LINE COUNT
          sts     INSA,ZL
          sts     INSA+1,ZH     ; INSERT LINE POINTER
          lds     A,IBCNT       ; NEW LINE COUNT
          brcs    LT            ; JMP IF NEW LINE # NOT OLD LINE NUMBER
          subi    A,4           ;SUI
          breq    LT1           ; TEST IF SHOULD DELETE NULL LINE
          ldi     c_tmp,4        ; ADI
          add     A,c_tmp
LT1:      sub     A,XL
          breq    LIN1          ; LINE LENGTHS EQUAL
          brcs    GT
;*
;*   EXPAND FILE FOR NEW OR LARGER LINE
;*
LT:       mov     XH,A          ; REMEMBER DIFFEENCE
          lds     A,IBCNT
          cpi     A,4           ; DON'T INSERT NULL LINE
          brne    PC+2          ; RZ
          ret
          mov     A,XH
          rcall   FULL
          lds     ZL,INSA     ;LHLD
          lds     ZH,INSA+1
          rcall   NMOV
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
          _XCHG
          sts     EOFA,ZL
          sts     EOFA+1,ZH
          adiw    XL,1
          rcall   RMOV
          rjmp    LIN1
;*
;*   CONTRACT FILE FOR SMALLER LINE
;*
GT:       com     A
          inc     A
          rcall   ADR
          rcall   NMOV
          _XCHG
          lds     ZL,INSA     ;LHLD
          lds     ZH,INSA+1
          breq    PC+2          ; CNZ
          rcall   LMOV
          ldi     c_tmp,1
          st      Z,c_tmp
          sts     EOFA,ZL
          sts     EOFA+1,ZH
;*
;*   INSERT CURRENT LINE INTO FILE
;*
LIN1:     lds     ZL,INSA     ;LHLD
          lds     ZH,INSA+1
          lds     A,IBCNT
          cpi     A,4
          brne    PC+2          ; RZ
          ret
;*
;*   INSERT CURRENT LINE AT ADDR HL
;*
IMOV:     ldi     YL,low(IBCNT)
          ldi     YH,high(IBCNT)
          ld      A,Y
          mov     XL,A
          ldi     XH,0
;*
;* COPY BLOCK FROM BEGINNING
;* HL IS DESTIN ADDR, DE IS SOURCE ADDR, BC IS COUNT
;*
LMOV1:    ld      A,Y
          st      Z,A
          adiw    YL,1
          adiw    ZL,1
          sbiw    XL,1
LMOV:     mov     A,XH
          or      A,XL
          brne    LMOV1
          ret
;*
;*  COPY BLOCK STARTING AT END
;*  HL IS DESTIN ADDR,DE IS SOURCE ADDR, BC IS COUNT
;*
RMOV1:    ld      A,Y
          st      Z,A
          sbiw    ZL,1
          sbiw    YL,1
          sbiw    XL,1
RMOV:     mov     A,XH
          or      A,XL
          brne    RMOV1
          ret
;*
;*   COMPUTE FILE MOVE COUNT
;*   BC GETS (EOFA) - (HL), RET Z SET MEANS ZERO COUNT
;*
NMOV:     lds     A,EOFA
          sub     A,ZL
          mov     XL,A
          lds     A,EOFA+1
          sbc     A,ZH
          mov     XH,A
          or      A,XL
          ret
;*
;*  ADD A TO HL
;*
ADR: 
		add ZL,A
		adc ZH,zero     
;          add     A,ZL
 ;         mov     ZL,A
  ;        brcs    PC+2          ; RNC
   ;       ret
    ;      inc     ZH
          ret
;*
;*  CHECK FOR FILE OVERFLOW,LEAVES NEW EOFA IN DE
;*  TAKES INCREASE IN SIZE IN A
;*
FULL:     lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
          rcall   ADR
          _XCHG
          ldi     ZL,low(MEMTOP+1)
          ldi     ZH,high(MEMTOP+1)
; PC no jump 9390
          call    DCMP1
; PC no branch 9882
          brcs    PC+3
          jmp    SOERR
          ret
;*
;% end of file BSM#EDIT
;*
;*
;*     THE  COMMANDS
;*
;*
;% start of file BSM#CMDS
;*
;*     DELETE COMMAND
;*
;*     DEL N1     DELETE N1
;*     DEL N1,N2  DELETE FROM N1 TO N2
;*     DEL ,N2    DELETE FROM FIRST LINE TO N2
;*     DEL N1,    DELETE FROM N1 TO LAST LINE
;*     DEL        DELETE ALL LINES
;*
; PC no jump 10005
CDEL:     call    GLARG         ; GET ARGUMENTS
; PC no jump 8169
          call    CRLF
;*
          lds     ZL,FIRST     ;LHLD
          lds     ZH,FIRST+1    ; DESTINATION
          _XCHG
          lds     ZL,LAST     ;LHLD
          lds     ZH,LAST+1     ; SOURCE
;*
          ld      A,Y
          cpi     A,1
          brne    PC+2          ; RZ
          ret
;*
          rcall   NMOV          ; BC:=(EOFA)-(HL)
;*
          _XCHG                 ; HL=DEST, DE=SOURCE, BC=COUNT
          rcall   LMOV          ; DELETE
;*
          ldi     c_tmp,1
          st      Z,c_tmp       ; EOF  NEW END OF FILE
          sts     EOFA,ZL
          sts     EOFA+1,ZH
          rjmp    ZAPER
;*
;*
;*     SCRATCH COMMAND
;*     CLEAR COMMAND
;*
;*   THIS COMMAND IS ALSO CALLED FROM THE INIT LOGIC
;*
;#ifdef _code_Revision_
;CCLEAR CALL FLCLA  CLOSE ALL FILES
; JMP ZAPER
;CSCR CALL FLCLA  CLOSE ALL FILES
;#endif
;*
;*
ZAPALL:   lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ldi     c_tmp,1
          st      Z,c_tmp
          sts     EOFA,ZL
          sts     EOFA+1,ZH
          sbiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; IN CASE CSCR IS EXECUTED AS A STATEMENT
;*
ZAPER:    lds     ZL,MEMTOP     ;LHLD
          lds     ZH,MEMTOP+1
          ldi     c_tmp,0
          st      Z,c_tmp
          sbiw    ZL,1
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
;*
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1     ; WIPE OUT DEFINITIONS
          adiw    ZL,1
          sts     STA,ZL
          sts     STA+1,ZH
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); CLEAR ERRSET TXA
          sts     ERRLN,ZL
          sts     ERRLN+1,ZH
;*
          eor     A,A
          sts     CONTF,A       ; CANT DO A 'CONT' AFTER 'CLEAR'
;#ifdef _code_Revision_
;CLR MATRIX ENTRY FLAG
;#endif
;*
; PC no jump 9409
          call    CFF           ; RESET PRINT FORMAT
; PC no jump 9380
          call    DFC
;*
          ldi     ZL,low(0x0034)
          ldi     ZH,high(0x0034); 26*2  ALLOCATE AND ZERO SYMTAB BUCKETS
; PC no jump 9760
          jmp     ASTAB
;*
;*
;*     RENUMBER COMMAND
;*
CREN:     ldi     ZL,low(10)
          ldi     ZH,high(10)
          sts     BEG,ZL
          sts     BEG+1,ZH      ; BEGINNING LINE NUMBER
          sts     DEL,ZL
          sts     DEL+1,ZH      ; DEFAULT INCREMENT
;*
; PC no jump 6800
          call    INTGER
          brcs    CREN1
          sts     BEG,ZL
          sts     BEG+1,ZH
;*
; PC no jump 9828
          call    SCOMA
          brne    CREN1
;*
; PC no jump 6785
          call    INTGER
; PC no branch 9019
          brcc    PC+3
          jmp     BAERR
          mov     A,ZH
          or      A,ZL
; PC no branch 9032
          brne    PC+3
          jmp    OBERR
          sts     DEL,ZL
          sts     DEL+1,ZH
;*  
; PC no jump 9794
CREN1:    call    GC
          cpi     A,13
; PC no branch 9003
          breq    PC+3
          jmp     BAERR
;*
;*   MAKE SURE ARGS WONT CAUSE OVERFLOW
;*
          ldi     YL,low(-1)
          ldi     YH,high(-1)   ; 177777Q  HIGHEST POSSIBLE LINE NUMBER
; PC no jump 9817
          call    FINDLN        ; WILL GIVE OBERR IF ARGS TOO BIG FOR PROGRAM
;*
;*  NOW WE HAVE BEG AND DEL SET UP
;*  BEGIN PASS 1 (CHANGING OF LINE NUMBER REFERENCES)
;*
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
;#ifdef _code_Revision_
;XRA A
; STA URFLAG  CLEAR UNRESOLVED REF FLAG
;#endif
;*
S_R0:     ld      A,Z
          cpi     A,1           ; EOF
          breq    S_R5          ; GOTO PASS 2
          adiw    ZL,1          ; PASS LINE LEN BYTE
          adiw    ZL,1
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; SAVE ADDR OF THE TEXT OF THIS LINE
;*
; PC no jump 9770
S_R1:     call    GCI           ; THE FIRST THING ON A LINE CAN'T BE AN LNRW
          cpi     A,13          ; CHECK FOR EOL
          breq    S_R0
; PC no jump 6769
S_R8:     call    LNUM          ; LN FOUND?
          brcs    S_R1          ; NO, SEARCH
;*
;* HERE WE HAVE FOUND A LINE NUM (IN HL) AND ADVANCED TXA
;* PAST IT
;*
          _XCHG                 ; PUT LINE NUMBER IN DE
; PC no jump 9784
          call    FINDLN        ; FIND THE NEW LINE NUMBER RETURNED IN 'NLN'
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); MAKE A ZERO LN INCASE OF UNRESOLVED REF
          brcs    S_R9          ; LINE NUMBER NOT FOUND, UNRESOLVED
          brne    S_R9          ; EXACT MATCH NOT FOUND, UNRESOLVED
;*
          lds     ZL,NLN     ;LHLD
          lds     ZH,NLN+1      ; THE NEW LINE NUMBER COMPUTED BY FINDLN
S_R9:     _XCHG                 ; NEW LINE NUMBER TO DE
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; TXA  NEW LN IS IN DE, HL <= TXA OF WHERE IT GOES + 2
          sbiw    ZL,1
          sbiw    ZL,1
; PC no jump 9310
          _DSTOR         ; UPDATE THE LINE NUMBER IN TEXT
          sts     TXA,ZL
          sts     TXA+1,ZH
;#ifdef _code_Revision_
; MOV A,D
; ORA E
; JNZ R8  GOTO R8 IF LN WAS NOT ZERO
; LDA URFLAG
; ORI 1
; STA URFLAG  FLAG SHOWING UNRESOLVED REF
;#endif
          rjmp    S_R8
;*
;*   PASS 2 OF RENUMBER (UPDATE THE LINE NUMBERS)
;*
S_R5:     lds     ZL,DEL     ;LHLD
          lds     ZH,DEL+1
          mov     XH,ZH
          mov     XL,ZL         ; INCREMENT
          lds     ZL,BEG     ;LHLD
          lds     ZH,BEG+1
          _XCHG
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
;*   
S_R6:     ld      A,Z           ; ACCA NOT USED UNTIL CALL TO ADR BELOW
          cpi     A,1
          brne    PC+2          ; RZ
          ret                   ; DONE, CHECK FOR UNRESOLVED REF
          push    ZL
          push    ZH            ; SAVE H FOR CALL TO ADR
          adiw    ZL,1          ; PASS LIN LEN, POINT TO LN
; PC no jump 9283
          _DSTOR         ; DE IS NEW LN, STORE IT
          pop     ZH
          pop     ZL            ; BACKUP, POINTER TO BEGINNING OF THIS LINE
          rcall   ADR           ; GET TO NEXT LINE, ACCA HAS DISTANCE FROM ABOVE
          _XCHG                 ; LAST LN (DE) TO HL
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; LINE NUMBER FOR NEXT LINE (LAST LN + DEL)
          _XCHG                 ; LN (HL) TO DE
          rjmp    S_R6
;#ifdef _code_Revision_
;*
;*
;R10 LDA URFLAG
; ORA A
; JNZ URERR
; JMP END1
;*
;*
;URFLAG DS 1  UNRESOLVED REF FLAG FOR REN COMMAND
;#endif 
;*
;*
;*     LIST COMMAND
;*
;*     LIST N1    LIST LINE N1
;*     LIST N1,   LIST FROM N1 TO LAST LINE
;*     LIST ,N2   LIST FROM FIRST LINE TO N2
;*     LIST N1,N2 LIST FROM N1 TO N2
;*     LIST       LIST ALL LINES
;*
; PC no jump 9780
CLIST:    call    GLARG         ; GET ARGUMENTS
; PC no jump 7944
          call    CRLF
; PC no jump 7941
          call    CRLF
          eor     A,A
          sts     LPHED,A       ; INITIALIZE INDENTATION COUNTER
          lds     ZL,FIRST     ;LHLD
          lds     ZH,FIRST+1    ;  GET ADDR OF FIRST LINE
;*
CL1:      ld      A,Z
          cpi     A,1           ; EOF
          brne    PC+2          ; RZ
          ret
;*
          ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1)
          rcall   UPPL          ; CONVERT LINE INTO TEXT (IN IBUF)
          adiw    ZL,1          ;  POINT TO BEGINING OF NEXT LINE TO LIST
;*
          push    ZL
          push    ZH
          ldi     ZL,low(IBUF1+5)
          ldi     ZH,high(IBUF1+5)
          ldi     c_tmp,'"'
          st      Z,c_tmp       ; TERMINATE LINE NUMBER
          ldi     ZL,low(IBUF1)
          ldi     ZH,high(IBUF1)
; PC no jump 9858
          call    PRNT          ; PRINT LINE NUMBER
          adiw    ZL,1          ; PASS THE "
;*
          ldi     XH,0xFE       ; -2
          lds     A,FORFG       ; THIS WILL CONTAIN A 2 IF LOGICLY TRUE
          or      A,A
          brne    LPADD
          mov     XH,A
LPADD:    lds     A,LPHED
          add     A,XH
          ldi     c_tmp,6        ; ADI
          add     A,c_tmp
          MOV     YL,A
; PC no jump 2223
          call    PTAB1
;*
; PC no jump 9829
          call    PRNTCR        ; PRINT THE STATEMENT
;IF SOLOS
; PC no jump 7924
          call    SPDCK         ; DO SPEED CONTROL
;ENDF
; PC no jump 7881
          call    CRLF          ; 
          pop     YH
          pop     YL
;*
          ldi     ZL,low(LAST)
          ldi     ZH,high(LAST) ; ADDR OF ADDR OF END OF LAST LINE
; PC no jump 9084
          call    DCMP
          _XCHG
          brne    PC+2          ; RZ
          ret
          rjmp    CL1
;*
;*
;*     EDIT COMMAND
;*
;*     EDIT        EDIT FIRST LINE
;*     EDIT N1     EDIT LINE N1
;*     EDIT N1,N2  EDIT LINE N1
;*     EDIT N1,    EDIT LINE N1
;*     EDIT ,N2    EDIT FIRST LINE
;*
; PC no jump 9699
CEDIT:    call    GLARG         ; GET LINE NUMBER ARGUMENTS
; PC no jump 7863
          call    CRLF          ; LINE FEED, CARRIAGE RETURN
;*
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ld      A,Z
          cpi     A,1
; PC no branch 8798
          brne    PC+3
          jmp    NPERR
;*
; PC no jump 8303
          call    READR         ; GET CURRENT DISPLAY ADDRESS INTO 'VDMAD'
          lds     ZL,FIRST     ;LHLD
          lds     ZH,FIRST+1
          ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1); PUT LINE IN IBUF1
          rcall   UPPL          ; DECODE LINE TO IBUF1 RETURNS C=# OF CHARS IN BUFFER
          dec     XL            ; GET RID OF THE CARRIAGE RETURN
          sbiw    YL,1          ; THAT UPPL PUT ON THE END
;*
          ldi     ZL,low(CEDRP)
          ldi     ZH,high(CEDRP); GET EDIT RETURN POINT
          push    ZL
          push    ZH            ; FOR RETURN FROM 'INLINE'
;*
          ldi     ZL,low(IBUF1)
          ldi     ZH,high(IBUF1); ADDR OF IBUF TO HL
          push    ZL
          push    ZH            ; BECAUSE OF ENTRY POINT USED BELOW
; PC no jump 8231
          jmp     INST2         ; ENTRY TO 'INLINE', C HAS CHARACTER COUNT
;*
CEDRP:    push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; SAVE TERMINATOR
          rcall   PP            ; ENCODE THE LINE
; PC no branch 8820
          brcc    PC+3
          jmp    LNERR         ; ONLY LINES WITH LINE NUMBERS!
          rcall   LINE          ; EDIT LINE IN
;*
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET TERMINATOR
;IF SOLOS
          cpi     A,10          ; LF
; PC no branch 7813
          brne    PC+3
          jmp    CRLF
CED07:    ldi     XH,10         ; LF  LINE FEED, CARRIAGE RETURN
; PC no jump 7782
          call    CHOUT
          ldi     XH,13
; PC no jump 7777
          call    CHOUT
          rjmp    ZAPER
;ENDF
;*
;*
;*
;*     CONTINUE COMMAND
;*
CCONT:    lds     A,CONTF
          or      A,A
          ldi     XL,low(0x4E43)
          ldi     XH,high(0x4E43)
; PC no branch 8813
          brne    PC+3
          jmp    ERROR
; PC no jump 7787
          call    CRLF
          eor     A,A
          sts     DIRF,A
          pop     ZH
          pop     ZL            ; RETURTN LINK
          pop     ZH
          pop     ZL            ; SAVED VALUE OF PROGRAM TXA
          sts     TXA,ZL
          sts     TXA+1,ZH
          pop     ZH
          pop     ZL
          pop     YH
          pop     YL
          pop     XH
          pop     XL            ; SAVED REGISTORS
          ldi     A,24          ; KCAN  THIS INCASE WE ARE RETURNING TO INLINE
          ret                   ; RETURN FROM PCHECK
;*
;*
;*    RUN COMMAND
;*
;#ifdef _code_Revision_
;CRUN LDA DIRF
; ORA A
; CNZ CRLF  NEW LINE IF NOT RUNNING
;*
;#endif
CRUN:     lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          push    ZL
          push    ZH
; PC no jump 9598
          call    GLARG         ; GET LINE NUMBER ARG
          pop     ZH
          pop     ZL
          ld      A,Z
          cpi     A,13          ; IF THERE WAS AN ARGUMENT THEN...
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          brne    PC+2          ; CZ
          rcall   ZAPER         ; ...DON'T CLEAR
;*
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ld      A,Z
          cpi     A,1           ; EOF  CHECK FOR NULL PROGRAM
; PC no branch 8692
          brne    PC+3
          jmp    NPERR
;*
          sbiw    ZL,1          ; PTR TO CR PRECEDING FIRST LINE TO BE EXECUTED
          sts     TXA,ZL
          sts     TXA+1,ZH      ; THE PLACE TO START RUNNING AT
          sts     RTXA,ZL
          sts     RTXA+1,ZH
          ldi     c_tmp,13
          st      Z,c_tmp
          eor     A,A
          sts     DIRF,A        ; CLEAR DIRECT MODE FLAG (RUN MODE NOW)
;#ifdef _code_Revision_
;STA MENT  CLEAR MATRIX ENTRY FLAG
;#endif
          sts     CONTF,A       ; CLEAR CONTINUE FLAG
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); CLEAR TIME/COUNT LIMIT
          sts     ITIM,ZL
          sts     ITIM+1,ZH
          sts     ITIM+1,ZL
          sts     ITIM+1+1,ZH
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
; PC no branch 65
          breq    PC+2
          rjmp    CRUN4         ; IF THERE WAS A LN ARG THEN SKIP FUNC DEF
;*
; PC no jump 9023
          call    CFF           ; RESET TO FREE FORM
; PC no jump 8994
          call    DFC
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); RESET ERROR TRAPPING
          sts     ERRLN,ZL
          sts     ERRLN+1,ZH
;*
;#ifdef _code_Revision_
; LHLD BOFA
; DCX H  POINT TO INITIAL CR WHICH PRECEEDS THE PROG BUFFER
; SHLD RTXA  SET THE READ STATEMENT DATA POINTER (INITIALY)
;#endif
;*
;*
;*  DEFINE FUNCTIONS, CHECK FOR FNEND BALANCE
;*
CRUN1:    ldi     XL,low(DEFRW*256+FNERW )
          ldi     XH,high(DEFRW*256+FNERW );  LOOK FOR DEF OR FNEND
          eor     A,A
; PC no jump 9191
          call    LSTAT
          brcs    CRUN4         ; DONE DEFINING
          cpi     A,0xA8        ; FNERW
; PC no branch 8660
          brne    PC+3
          jmp    FDERR
;*
;*    NOW WE KNOW IT MUST HAVE BEEN A DEFRW
;*
          ldi     XH,0x84       ; FNRW
; PC no jump 9416
          call    EATC
; PC no jump 9167
          call    FNAME
; PC no jump 6139
          call    STLK
          ldi     XL,low(0x4444)
          ldi     XH,high(0x4444); ERROR IF NAME NOT CREATED
; PC no branch 8698
          brcs    PC+3
          jmp    ERROR
          push    ZL
          push    ZH            ; SYMTAB PTR
          ldi     ZL,low(2)
          ldi     ZH,high(2)
; PC no jump 9335
          call    ASTAB
; PC no jump 9387
          call    EATLP
          _XCHG
          pop     ZH
          pop     ZL
; PC no jump 8983
          _DSTOR         ; SAVE TXA IN SYMTAB
;*
;*     EAT UP DEFINITON
;*
; PC no jump 9169
          call    FEND
          rjmp    CRUN1
;*   
;*   
CRUN4:    lds     ZL,FIRST     ;LHLD
          lds     ZH,FIRST+1    ; STARTING TXA
          adiw    ZL,1
          adiw    ZL,1
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
; PC no jump 7642
          call    CRLF
;* 
.if 0  
          ldi     ZL,low(CRM)
          ldi     ZH,high(CRM)  ; DO COPYRIGHT CHECK
          eor     A,A
          ldi     XH,'-'        ; CRML
CRUNC:    ld      c_tmp,Z
          adc     A,c_tmp       ; ONES CHECKSUM IS LEFT CIRCULAR
          adiw    ZL,1
          dec     XH            ; LENGTH
          brne    CRUNC
;*   
          lds     ZL,STA     ;LHLD
          lds     ZH,STA+1      ; SYMBOLTABLE FREE POINTER (SNIKER...SNIKER)
          ldi     YL,low(STKTOP)
          ldi     YH,high(STKTOP); TOP OF STACK
          _XCHG
;CKSUM EQU $+1
          cpi     A,0x87        ; IS IT WHAT IT SHOULD BE??
          breq    CRUNY         ; YES
          _XCHG                 ; NO, THEN HIS VARS WILL CLOBBER THE STACK
          ldi     XL,low(SSIZE-56)
          ldi     XH,high(SSIZE-56)
CRUNY:    ldi     XL,low(SSIZE)
          ldi     XH,high(SSIZE); STACK SIZE
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; BOTTOM OF STACK POINTER
          sbiw    ZL,1          ; LESS ONE
          out     SPL,ZL        ;SPHL
          out     SPH,ZH
          sts     SPTR,ZL
          sts     SPTR+1,ZH
.endif
; not sure why stack is reset other than as part of the
; anti reverse engineering code
;*   
          ldi     A,13          ; CLEAR IF TERM
          sts     IFTERM,A
;*   
          rjmp    ILOOP         ; GO TO THE INTERPRETER DRIVER
;*
;*
;*     SET COMMAND
;*
; PC no jump 9341
CSET:     call    GCI           ; GET THING TO SET
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
; PC no jump 6242
          call    PFIXE         ; GET EXPRESSION AND FIX TO DE
CSET2:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          push    YL
          push    YH            ; SAVE RESULT
;*   
          cpi     A,0x96        ; LNRW  ALL 'SET' WORDS ARE BELOW 'LNRW'
; PC no branch 8603
          brcs    PC+3
          jmp    BSERR
          subi    A,0x90    ;SUI
; PC no branch 8598
          brcc    PC+3
          jmp    BSERR         ; NOT A 'SET' WORD
;*   
          lsl     A             ; MUL TIMES 2
          ldi     ZL,low(STBL*2)
          ldi     ZH,high(STBL*2) ; ADDR OF SET TABLE
          rcall   ADR           ; HL=HL+A
; PC no jump 8786
CSET1:    call    pgm_LHLI          ; HL=(HL)
          mov     DPL,ZL     ; XTHL
          mov     DPH,ZH
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; EXPRESSION RESULT TO HL, HL TO STACK
          mov     A,ZL          ; A GETS LOW OF EXPRESSION RESULT
          ret                   ; CALL SETTING ROUTINE, IT WILL RETURN FOR US
;*   
;*   
SETLL:    mov     A,ZH
          or      A,A
; PC no branch 8580
          breq    PC+3
          jmp    BSERR         ; WAY TOO BIG
          ldi     A,LINMAX     ; 0x84
          cp      A,ZL
; PC no branch 8528
          brcc    PC+3
          jmp    OBERR
          mov     A,ZL
          sts     LINLEN,A
          ret
;*   
;*   
SETML:    lds     A,DIRF
          or      A,A
; PC no branch 8562
          brne    PC+3
          jmp    BSERR
;*   
          _XCHG                 ; DE HAS NEW LIMIT
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ldi     XL,low(LINMAX+1)
          ldi     XH,high(LINMAX+1)
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; LOWEST POSSIBLE MEMORY LIMIT IS BOFA+LINMAX+1
          _XCHG                 ; HL HAS NEW LIMIT, DE HAS MIN LIMIT
; PC no jump 8734
          _HDCMP         ; HL-DE TEST
; PC no branch 8501
          brcc    PC+3
          jmp    OBERR         ; TOO SMALL
;MEMAX EQU $+1
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000); THE HIGEST POSSIBLE MEMORY ADDRESS FOR BASIC
          adiw    YL,1
; PC no jump 8724
          _HDCMP
; PC no branch 8491
          brcs    PC+3
          jmp    OBERR
          sts     MEMTOP,ZL
          sts     MEMTOP+1,ZH
          rjmp    ZAPER         ; THIS MAKES TI TAKE EFFECT RIGHT-A'-WAY
;*
; IF SOLOS
;*
;*
SETDS:    ldi     XH,8          ; SET DISPLAY SPEED
          rjmp    ESCSEQ        ; DO ESCAPE SEQ AND RETURN
;*
;*
SETDB:    ldi     XH,7          ; DISPLAY BYTE
          rjmp    ESCSEQ
;*
;*
SETIP:    lds     ZL,XIPORT     ;LHLD
          lds     ZH,XIPORT+1   ; SET INPUT PSEUDO PORT
          st      Z,A
          ret
;*
;*
SETOP:    lds     ZL,XOPORT     ;LHLD
          lds     ZH,XOPORT+1   ; SET OUTPUT PSEUDO PORT
          st      Z,A
          ret
;*
;*   DO A SOLOS ESCAPE SEQUENCE
;*
ESCSEQ:   push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; AN ESCAPE SEQU. JUST LIKE IN THE BOOK
          push    XL
          push    XH
          ldi     XH,27         ; KESC
; PC no jump 7956
          call    ZOUT
          pop     XH
          pop     XL
; PC no jump 7952
          call    ZOUT
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          mov     XH,A
; PC no jump 7947
          jmp     ZOUT          ;   ...AND RETURN
; ENDF
;*
;*
;*     TAPE ON COMMAND
;*
; PC no jump 6112
CTON:     call    PFIXE
          mov     A,YH
          or      A,A
; PC no branch 8429
          breq    PC+3
          jmp    OBERR         ; WOW!
          or      A,YL          ;  MOVE E TO A AND TEST
; PC no branch 8425
          brne    PC+3
          jmp    OBERR
          sts     CSFID,A       ; FOR TTON
          cpi     A,3
; PC no branch 2923
          brcc    PC+3
          jmp    TTON
; PC no jump 8414
          jmp     OBERR
;*
;*
;% end of file BSM#CMDS
;*
;*
;*     THE  PROGRAM  STORAGE  COMMANDS
;*
;*
;% start of file BSM#DPSS
;*
;*
;*   SAVE
;*
; PC no jump 2719
CSAVE:    call    GFNX          ; GET FILE NAME
;*
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          ld      A,Z
          cpi     A,1           ; EOF  TEST FOR EMPTY PROGRAM
; PC no branch 8372
          brne    PC+3
          jmp    NPERR
;*
          ldi     ZL,low(OFT+OFTEZ)
          ldi     ZH,high(OFT+OFTEZ)
          sts     OFCB,ZL
          sts     OFCB+1,ZH
          ldi     A,'C'         ; BTSCP  DEFAULT TYPE FOR CREATING
          sts     CFT,A
;*
; PC no jump 9156
          call    GC            ; GET SAVE TYPE
          cpi     A,13
          breq    PSAVE         ; TAKE DEFAULT
;*
          cpi     A,','
; PC no branch 8424
          breq    PC+3
          jmp    BSERR
;*
; PC no jump 9148
          call    GCI
; PC no jump 9145
          call    GCI
          cpi     A,'T'         ; TEXT TYPE?
          breq    NEX0
          cpi     A,'C'         ; SEMI-COMPILED TYPE
; PC no branch 8408
          breq    PC+3
          jmp    BSERR         ; THEN WHAT???
NEX0:     sts     CFT,A         ; WHAT TO CREATE IT AS
;*
;*  OPEN FILE
;*
NEX1:     cpi     A,'T'         ; BTSCP
          breq    ASAVE
;*   
;*   
; PC no jump 9119
PSAVE:    call    GC
          cpi     A,13
; PC no branch 8392
          breq    PC+3
          jmp    BSERR
;*   
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1     ; CALC PROGRAM SIZE
          sts     tBlkST,ZL
          sts     tBlkST+1,ZH
          _XCHG
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
; PC no jump 5968
          _DSUB
          adiw    ZL,1
          sts     CBLKS,ZL
          sts     CBLKS+1,ZH
; PC no jump 2832
          call    TTON
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -2472
          call    cWRBLK
; PC no branch 8275
          brcc    PC+3
          jmp    WTERR
          rjmp    END1
;*
;*
; PC no jump 9079
ASAVE:    call    GC
          cpi     A,13
; PC no branch 8352
          breq    PC+3
          jmp    BSERR
;*
          sts     CUFID,A
          ldi     A,2
; PC no jump 2967
          call    S_156B
; PC no jump 2591
          call    CLOFCB
;*
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
ASAV0:    ldi     YL,low(IBUF)
          ldi     YH,high(IBUF)
; PC no jump -2053
          call    UPPL          ; UN-PREPROCESS LINE
          adiw    ZL,1          ; POINT TO NEXT LINE
;*
          push    ZL
          push    ZH
          mov     YL,XL         ; MOVE C TO DE
          ldi     YH,0
          sbiw    YL,1          ; FORGET THE CR ON THE END
          ldi     ZL,low(IBUF)
          ldi     ZH,high(IBUF)
          ldi     XL,low(0x0001)
          ldi     XH,high(0x0001)
; PC no jump 2793
          call    FLW1I         ; WRITE LINE OUT AS A SINGLE ITEM
          pop     ZH
          pop     ZL
;*
          ld      A,Z           ; TEST FOR EOF
          cpi     A,1
          brne    ASAV0         ; MORE TO DO
;*
          eor     A,A
          sts     XEQFG,A
          rjmp    GETC
;*
;*
;*     "GET", "XEQ" AND "APPEND"
;*
CXEQ:     ldi     A,0xFF        ; SET XEQ FLAG
          sts     XEQFG,A
;*
;% Append file command
;*
          eor     A,A           ; CLEAR APPEND FLAG
          sts     APPFG,A
          rjmp    GET1
;*
;% Append file command
;% this keyword slot may be used for cat
;*
CAPP:     ldi     A,0xFF        ; SET APPEND FLAG
          sts     APPFG,A
          rjmp    L_0A15
;*
;% Get file command
;*
CGET:     eor     A,A           ; CLEAR APPEND FLAG
          sts     APPFG,A
L_0A15:   eor     A,A           ; % part of cassete system
          sts     XEQFG,A
;*
; PC no jump 2534
GET1:     call    GFNX
;*
          ldi     ZL,low(OFT+OFTEZ)
          ldi     ZH,high(OFT+OFTEZ); GET FILE NAME
          sts     OFCB,ZL
          sts     OFCB+1,ZH
; PC no jump 8985
          call    GC
          cpi     A,13
          ldi     A,'C'
          breq    L_0A37
; PC no jump 8980
          call    GCI
          cpi     A,','
; PC no branch 8248
          breq    PC+3
          jmp    BSERR
; PC no jump 8972
          call    GCI
;*
L_0A37:   cpi     A,'C'         ; BTSCP  SEMI-COMP?
          breq    PGET
          cpi     A,'T'
; PC no branch 8235
          breq    PC+3
          jmp    BSERR
;*
;*  GET FROM A TEXT FILE
;*
AGET:     rcall   ZAPER         ; CLEAR VARIABLES
          lds     A,APPFG       ; APPENDING?
          or      A,A
          brne    PC+2          ; CZ
          rcall   ZAPALL        ; NO, ERASE OLD PROGRAM
          ldi     A,1
          sts     CUFID,A
; PC no jump 2840
          call    S_156B
; PC no jump 2464
          call    CLOFCB
;*
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; AVE WHERE WE ARE ON INPUT LINE
          push    ZL
          push    ZH
;*
AGET0:    ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1)
; PC no jump 2740
          call    FLR1I         ; READ ONE ITEM (ONE LINE)
          breq    GETC0         ; EOF, DONE
; PC no jump -2491
          call    PP            ; PRE-PROCESS THE LINE
; PC no branch 8173
          brcc    PC+3
          jmp    LNERR         ; MUST HAVE LINE NUMBERS!!
          rcall   LINE          ; EDIT LINE INTO PROGRAM
          rjmp    AGET0         ; MORE LINES?
;*
;*  GET FROM A SEMI-COMPILED FILE
;*
PGET:     lds     A,APPFG       ; TEST FOR APPENDING REQUESTED
          or      A,A
; PC no branch 8088
          breq    PC+3
          jmp    CAERR         ; CAN'T APPEND FROM A SEMI-COMPILED PROGRAM FILE
;*
          rcall   ZAPALL        ; WIPE SLATE CLEAN
; PC no jump 2636
          call    TTON
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1     ; ADDR OF WHERE TO PUT IT
          _XCHG                 ; TO DE
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -2675
          call    cRDBLK
; PC no branch 8081
          brcc    PC+3
          jmp    RDERR
          ldi     YL,low(-1)
          ldi     YH,high(-1)
; PC no jump 8913
          call    FINDLN
          _XCHG
          lds     ZL,MEMTOP     ;LHLD
          lds     ZH,MEMTOP+1
; PC no jump 8334
          _HDCMP
; PC no branch 8833
          brcc    PC+3
          jmp    SOERR
          _XCHG
          sts     EOFA,ZL
          sts     EOFA+1,ZH
          rcall   ZAPER
          rjmp    GETC_99
;*
;*  RETURNS
;* 
GETC0:    pop     ZH
          pop     ZL            ; GET TXA
          sts     TXA,ZL
          sts     TXA+1,ZH      ; RESTORE
; PC no jump 2732
GETC:     call    FLCLZ         ; CLOSE FILE
;*
GETC_99:  lds     A,XEQFG       ; TEST FOR XEQ
          or      A,A
; PC no branch -669
          breq    PC+2
          rjmp    CRUN          ; RUN THE PROGRAM (Z MUST BE 1 FOR CRUNX)
          rjmp    END1          ; ALL DONE
;*
;* 
;% end of file BSM#DPSS
;*
;*
;*     THE  "BASIC"  STATEMENTS
;*
;*
;% start of file BSM#STM1
;*
;*
;*   LET STATEMENT
;*
;*
; PC no jump 5268
LET:      call    VAR           ; CHECK FOR VARIABLE
          push    ZL
          push    ZH            ; SAVE VAR ADDRESS
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; SAVE VARIABLE FLAG
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; GET TEXT POINTER
          ld      A,Z           ; NEXT CHR
          cpi     A,0xD4        ; EQRW  SHOULD BE AN EQUALS
; PC no branch 8104
          breq    PC+3
          jmp    BSERR         ; ERROR IF NOT
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; EAT THE "="
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET BACK FLAG FROM VAR
          brne    LET0          ; NOT A STRING
;*
;*   REMEMBER HL ARE ON THE STACK TO STAS1 AND EXPRB
;*   FROM ABOVE
;*
          pop     ZH
          pop     ZL
          ldi     A,0           ; FLAG FOR STRING PEOPLE
; PC no jump 3573
          call    STASS         ; DO STRING ASSIGNMENT
          rjmp    LET1
;*
; PC no jump 2921
LET0:     call    EXPRB
          pop     YH
          pop     YL            ; DESTINATION ADDRESS
; PC no jump 8684
          call    POPA1         ; COPY EXPR VALUE TO VARIABLE
;*
; PC no jump 8813
LET1:     call    SCOMA
          breq    PC+2          ; RNZ
          ret
          rjmp    LET           ; MULTI-ASSIGNMENTS PER LET STATEMENT
;*
;*
;*
;*
;*   FOR STATEMENT
;*
;*
; PC no jump 5648
SFOR:     call    DIRT
; PC no jump 5220
          call    VAR           ; CONTROL VARIABLE
; PC no branch 8024
          brne    PC+3
          jmp    TYERR         ; STRING TYPE IS ERROR
          push    ZL
          push    ZH            ; CONTROL VARIABLE VALUE ADDRESS
          ldi     XH,0xD4       ; EQRW
; PC no jump 8767
          call    EATC
; PC no jump 2892
          call    EXPRB         ; INITIAL VALUE
          pop     YH
          pop     YL            ; CONTROL VARIABLE VALUE ADDRESS
          push    YL
          push    YH            ; SAVE FOR LATER
; PC no jump 8654
          call    POPA1         ; SETS INITIAL VALUE
          ldi     XH,0x81       ; TORW  RESERVED WORD VALUE FOR 'TO'
; PC no jump 8754
          call    EATC
; PC no jump 2879
          call    EXPRB         ; LIMIT VALUE COMPUTATION
          ldi     A,0x80        ; STEPRW
; PC no jump 8775
          call    SCANC         ; CHECK NEXT CHARACTER FOR POSSIBLE STEP EXPRES
          brcc    FOR1          ; JNC FOR1
;*
;*  USE STEP OF 1
;*
          ldi     ZL,low(FPONE)	; fix this in the equates a 
          ldi     ZH,high(FPONE); the start value is odd
; PC no jump 8611
          call    pgm_PSHAS      ; call a pgm copy as it returns through
								 ; pgm_Vcopy
          rjmp    FOR2
;*
;*  HERE COMPUTE STEP VALUE
;*
; PC no jump 2859
FOR1:     call    EXPRB         ; THE STEP VALUE
FOR2:     ldi     YL,low(0xFFFB)
          ldi     YH,high(0xFFFB); -5 ALLOCATE SPACE ON STACK FOR TXA, CONTROL VAR
; PC no jump 8633
          call    PSHCS
          _XCHG                 ; SAVE DE
; PC no jump 8381
          call    JOE           ; RETURNS TEXT ADDRESS
; PC no branch 1565
          brcc    PC+2
          rjmp    CSERR         ; ILLEGAL FOR FOR STATEMENT TO BE LAST IN PROGRAM
          _XCHG                 ; DE NOW HAS LOOP TEXT ADDRESS AND HL HAS CONTROL
          st      Z,YH          ; HIGH ORDER TEXT ADDRESS BYTE
          sbiw    ZL,1
          st      Z,YL          ; LOW ORDER TEXT ADDRESS BYTE
          pop     XH
          pop     XL            ; CONTROL VARIABLE ADDRESS
          push    ZL
          push    ZH            ; TEXT ADDRESS FOR TEST
          sbiw    ZL,1
          st      Z,XH          ; HIGH ORDER BYTE OF CONTROL VARIABLE ADDRESS
          sbiw    ZL,1
          st      Z,XL          ; LOW ORDER BYTE OF CONTROL VARIABLE ADDRESS
          push    XL
          push    XH            ; SAVE VARIABLE ADDRESS FOR TEST
          sbiw    ZL,1
          ldi     c_tmp,1
          st      Z,c_tmp       ; FRTYPE  SET CONTROL STACK ENTRY TYPE TO 'FOR'
          ldi     XL,low(FPSIZ-1+4)
          ldi     XH,high(FPSIZ-1+4); POINT TO SIGN OF STEP
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          push    ZL
          push    ZH            ; SAVE FOR TEST
;*
;*  NOW SCAN TO THE MATCHING NEXT STATEMENT TO CHECK FOR '0' E
;*
          _XCHG                 ; TXA POINTS INTO NEXT STATEMENT, WE MUST BACK IT
          in r25,SREG
          sbiw    ZL,4          ; BACK UP OVER LINE NUMBER, COUNT AND CARRIAGE RETURN
          out SREG,r25
;          sbiw    ZL,1
 ;         sbiw    ZL,1
  ;        sbiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; DO THIS SO LSTAT WORKS
          eor     A,A           ; USED BY LSTAT
          mov     XH,A          ; INITIALIZE BALANCE COUNT
;*
FOR3:     inc     XH            ; BUMP 'FOR' COUNT
;*
FOR4:     push    XL
          push    XH
          ldi     XL,low(0x999B)
          ldi     XH,high(0x999B); 0x999B FORRW*256+NEXTRW
; PC no jump 8443
          call    LSTAT
          ldi     XL,low(0x4353)
          ldi     XH,high(0x4353); 'CS'
; PC no branch 7950
          brcc    PC+3
          jmp    EOFERR
;*
;*  HERE IF WE FIND A FOR, BUMP COUNT AND CONTINUE SEARCH
;*
          cpi     A,0x99        ; FORRW
          pop     XH
          pop     XL
          breq    FOR3  ; FOUND ANOTHER FOR
;*
;*  HERE WE FOUND A NEXT, SEE IF IT IS THE ONE WE NEED
;*
          dec     XH
          brne    FOR4          ; JUMP IF NEED MORE
;*
;*   FOUND THE MATCHING NEXT
;*
; PC no jump 8368
          call    NAME1         ; LOOK AT VARIABLE NAME (IF ANY)
          rjmp    NEX1A
;*
;*
;*
;*
;*   NEXT STATEMENT
;*
;*
;*
; PC no jump 5525
NEXT:     call    DIRT
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; CONTROL STACK ADDRESS
          adiw    ZL,1
          ld      A,Z           ; STACK ENTRY TYPE BYTE
          dec     A             ; MUST BE FOR TYPE (=1) ELSE ERROR
; PC no branch 1494
          breq    PC+2
          rjmp    CSERR         ; IMPROPER NESTING ERROR
          adiw    ZL,1          ; CONTROL STACK POINTER TO CONTROL VARIABLE ADDRESS
          push    ZL
          push    ZH
; PC no jump 8348
          call    NAME1         ; CHECK VARIABLE, IN CASE USER WANTS CHECK
          _XCHG
          pop     ZH
          pop     ZL            ; ADDRESS OF CONTROL VARIABLE ADDRESS
          brcs    NEXT1         ; NO VARIABLE
; PC no jump 8115
          call    DCMP          ; COMPARE ADDRESSES
; PC no branch 1478
          breq    PC+2
          rjmp    CSERR         ; IMPROPER NESTING IF NOT THE SAME
;*
NEXT1:    ld      XL,Z          ; ADDRESS OF CONTROL VARIABLE ADDRESS
          adiw    ZL,1
          ld      XH,Z
          adiw    ZL,1
          push    ZL
          push    ZH            ; TEXT ADDRESS
          push    XL
          push    XH            ; VARIABLE ADDRESS
          ldi     YL,low(0x0006)
          ldi     YH,high(0x0006); FPSIZ-2+2  LEAVE THIS AS IS
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; POINT TO SIGN OF STEP VALUE
          push    ZL
          push    ZH            ; AND SAVE FOR LATER
          adiw    ZL,1
          mov     YH,XH         ; HIGH ORDER BYTE OF VAR ADDRES
          mov     YL,XL
;*
;*     BC=DE+HL
;*
; PC no jump 3577
          call    FADD          ; DO INCREMENT
NEX1A:    pop     ZH
          pop     ZL            ; SIGN OF STEP VALUE
          ld      A,Z           ; SIGN (1=NEG)
          ldi     YL,low(FPSIZ+1)
          ldi     YH,high(FPSIZ+1); 
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; PUTS LIMIT ADDRESS IN HL
          pop     YH
          pop     YL            ; ADDRESS OF VAR ADDRESS
          or      A,A           ; SET CONDITIONS BASED ON SIGN OF STEP VALUE  STILL
          breq    NEXT2
          _XCHG                 ; IF POS STEP, SWITCH TEST ORDER
;*
; PC no jump 3493
NEXT2:    call    RELOP         ;  SET CONDITIONS (CARRY SET MEANS STOP LOOP)
          pop     ZH
          pop     ZL            ; ADDRESS TEXT ADDRESS
          brcc    NEXT3         ; JUMP IF MUST CONTINUE LOOP
;*
;*  HERE TEMINATE LOOP
;*
          sbiw    ZL,1
          ld      YH,Z
          sbiw    ZL,1
          ld      YL,Z          ; DE HAS CONTROL VARIABLE ADDRESS
          ldi     XL,low((2*FPSIZ)+3)
          ldi     XH,high((2*FPSIZ)+3); 0x0F
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; UNDO THE LOOP FROM THE STACK!!
; PC no jump 8434
          jmp     VCOPY         ; MAKE CTRL-VAR EQU TO LIMIT ALSO RETURN
NEXT3:    ld      YL,Z
          adiw    ZL,1
          ld      YH,Z
;*
;*  PREPARE TO ITERATE, SKIPPING NORMAL JUNK ON END TEST AT IL
;*
NEXT4:    _XCHG                 ; GET BACK REAL TEXT ADDRESS
          sts     TXA,ZL
          sts     TXA+1,ZH      ; NOTE-----  COMMENTS ALLOWED AFTER NEXT STATEMENT
          pop     ZH
          pop     ZL            ; EAT RETURN LINK
          rjmp    ILOOP         ; RETURN TO DISPATCHER SKIPPING JOE CALL THERE
;*
;*
;*
;*
;*   IF STATEMENT
;*
;*
;*
; PC no jump 8073
SIF:      call    STEST         ; RETURNS Z SET IF STRING EXPRESSION NEXT
          breq    SIF3
;*  
;*  NUMERIC IF CASE
; PC no jump 2689
          call    EXPRB
; PC no jump 8449
          call    POPFP         ; REMOVE VALUE FROM ARG STACK TO FPSINK
;*  
SIF0:     ldi     XH,0x83       ; THENRW
; PC no jump 8553
          call    EATC
          ldi     XH,1          ; COUNT TO USE IF CONDITION FALSE
          lds     A,FPSINK      ; CHECK FOR NON-ZERO VALUE
          or      A,A
; PC no branch 87
          brne    PC+2
          rjmp    SIF9          ; GOT TO LOOK FOR ELSE
;*
;*   SUCCESSFUL IF STATEMENT
SIF00:    ldi     A,0x82        ; ELSERW
          sts     IFTERM,A      ; A LEGAL TERMINAFOR FOR NEXT STATEMENT
;*
;*  TEST SUCCEEDED
;*
; PC no jump 5561
          call    LNUM          ; CHECK IF LINE NUMBER IS DESIRED ACTION
; PC no branch 124
          brcs    PC+2
          rjmp    GOTO1         ; DO A GOTO
          rjmp    ISTA0         ; INTERPRET REST OF STATEMENT AS NEW STATEMENT
;*
;*
;*  STRING IF CASE
;*
SIF3:     lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          push    ZL
          push    ZH            ; SAVE TEMP STACK POINTER
;*
;*   GET LEFT STRING TO TEMP STACK
;*
          sts     LHSBA,ZL
          sts     LHSBA+1,ZH    ; BASE ADDR OF LEFT STRING
; PC no jump 3082
          call    SEXPG
          push    ZL
          push    ZH            ; SIZE OF LEFT STRING
;*
;*   GET RELATIONAL OPERATOR
;*
; PC no jump 8534
          call    GCI
          cpi     A,0xD6        ; GTRW+1  CHECK IF LEGAL RELATIONAL
; PC no branch 7802
          brcs    PC+3
          jmp    BSERR
          cpi     A,0xD0        ; GERW
; PC no branch 7797
          brcc    PC+3
          jmp    BSERR
;*
;*   COMPUTE RELOP ACTION ROUTINE ADDRESS
;% adjust this code for lpm etc ...
          ldi     ZL,low(OPTAB)
          ldi     ZH,high(OPTAB)
          subi    A,0xC8           ;SUI; TOKCM
; PC no jump 2867
          call    OPADR
          adiw    ZL,1
; PC no jump 7983
          call    LHLI
          adiw    ZL,1
          adiw    ZL,1
          adiw    ZL,1
          pop     YH
          pop     YL            ; LHS SIZE
          push    ZL
          push    ZH            ; ACTION ROUTINE ADDR
          push    YL
          push    YH            ; SIZE OF LHS
;*
;*   GET RIGHT HAND STRING
;*
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          sts     RHSBA,ZL
          sts     RHSBA+1,ZH    ; RHS BASE ADDR
; PC no jump 3041
          call    SEXPG
          _XCHG                 ; DE HAS RHS SIZE
          pop     XH
          pop     XL            ; LHS SIZE
;*
;*  COMPARE
;*
; PC no jump 8710
          call    SCOMP
;*
;*  CONDITION CODES ARE SET, NOW DISPATCH TO ACTION ROUTINE
;*
SIF6:     ldi     ZL,low(SIF7)
          ldi     ZH,high(SIF7) ; SIF7  ACTION ROUTINE RETURN LINK
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          push    ZL
          push    ZH            ; PUSH BACK ACTION ROUTINE ADDR
          ldi     XL,low(FPSINK)
          ldi     XH,high(FPSINK); ADDR OF TEMP
          ret                   ; CALL ACTION ROUTINE
;*
;*
;*    RETURN HERE FROM ACTION ROUTINE WITH FPSINK
;*    SET ACCORDING
;*
SIF7:     pop     ZH
          pop     ZL            ; ORIGINAL VALUE OF TSTKA
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          rjmp    SIF0
;*
;*
;*
;*      COME HERE IF TEST FAILED, LOOK FOR POSSIBLE
;*    MATCHING ELSE
;*
SIF90:    inc     XH            ; IF-ELSE BALANCE COUNTER
;*
SIF9:     lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
SIF14:    ld      A,Z
          cpi     A,0x96        ; LNRW  SKIP LN'S
          brne    SIF13
          adiw    ZL,1          ; SKIP LN
          adiw    ZL,1          ; SKIP LOW
          adiw    ZL,1          ; SKIP HIGH
          sts     TXA,ZL
          sts     TXA+1,ZH
          rjmp    SIF14
;*
;*
SIF13:    cpi     A,13
          brne    PC+2          ; RZ
          ret                   ; WAS NO MATCHING ELSE CLAUSE
          adiw    ZL,1          ; EAT THE CHAR, NOW THAT IT ISN'T A CR
          sts     TXA,ZL
          sts     TXA+1,ZH
          cpi     A,0x9C        ; IFRW
          breq    SIF90
          cpi     A,0x82        ; ELSERW
          brne    SIF14
          dec     XH
          brne    SIF14
          rjmp    SIF00         ; MATCHING ELSE FOUND
;*
;*
;*
;*
;*
;*   GOTO STATEMENT
;*
;*
;*
; PC no jump 5437
GOTO:     call    LNUM          ; RETURNS INTEGER IN HL IF LINE NUMBER PRESE
; PC no branch 7700
          brcc    PC+3
          jmp    BSERR         ; SYNTAX ERROR IF NO LINE NUMBER
;*
; PC no jump 5275
GOTO1:    call    DIRT
          _XCHG                 ; FINDLN WANTS LN IN DE
; PC no jump 8496
GOTO3:    call    FNEQLN        ; RETURNS TEXT ADDRESS  POINTS TO COUNT VALUE
;*
GOTO2:    adiw    ZL,1
          adiw    ZL,1
          adiw    ZL,1          ; ADVANCE TEXT POINTER PAST LINE NUMBER AND COUNT
          sts     TXA,ZL
          sts     TXA+1,ZH
          pop     ZH
          pop     ZL            ; EAT RETURN LINK
          rjmp    ILOOP
;*
;*
;*     ERRCLR
;*
SERRC:    ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     ERRLN,ZL
          sts     ERRLN+1,ZH
          ret
;*
; PC no jump 5407
SERRS:    call    LNUM          ; GET THE LINE NUMBER
; PC no branch 7670
          brcc    PC+3
          jmp    BSERR         ; MUST HAVE LINE NUMBER
          _XCHG
; PC no jump 8469
SES0:     call    FNEQLN        ; THIS ENTRY USED BY "ON2"
          sts     ERRLN,ZL
          sts     ERRLN+1,ZH    ; SAVE UNTIL AN ERROR OCCURS
          ret                   ; ALL DONE
;*
;*
;*     ON STATEMENT
;*
; PC no jump 5289
ON:       call    PFIXE         ; NFIXE  GET VAL OF EXPR (DIRT IS CALLED LATER)
;% revised --added-- code
; MOV A,B  B#0 ==> NEG RESULT
; ORA A
; JNZ NEXTS  FALL TRU TO NEXT STATEMENT FOR NEG NUMBERS
; ORA D  NOTE: A IS ZERO
; JNZ NEXTS  TOO BIG, FALL THRU TO NEXT STATEMENT
;%
;*
; PC no jump 8381
          call    GCI           ; GET STATEMENT AFTER THE EXPRESSION (GOTO, GOSUB, ETC)
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; SAVE FOR ON2
          push    YL
          push    YH            ; SELECTOR
;*
; PC no jump 5383
ON1:      call    LNUM
; PC no branch 7646
          brcc    PC+3
          jmp    BSERR
          pop     XH
          pop     XL            ; SELECTOR
          dec     XL            ; LOW PART
          breq    ON2           ; THIS IS THE ONE WE WANT
          push    XL
          push    XH            ; SAVE FOR NEXT TIME'ROUND
; PC no jump 8359
          call    GC            ; TEST FOR CR
          cpi     A,13
          breq    ON9           ; FALL THRU
          cpi     A,0x97        ; EOSRW  OR EOS
          brne    ON1
;*
;*  NO LINE NUMBER SELECTED, FALL THRU TO NEXT STATEMENT
ON9:      pop     XH
          pop     XL
          pop     XH
          pop     XL            ; CLEAN STACK
          rjmp    NEXTS         ; FALL THROUGH TO NEXT STATEMENT
;*
;*
;*   HAVE DESTINATION LINE NUMBER IN HL
;*
ON2:      _XCHG                 ; DEST LN TO DE
; PC no jump 5196
          call    DIRT
          rcall   NEXTS         ; POINT TO NEXT LINE (FOR THOSE WHO CARE, LIKE GOSUB)
;*
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET STATEMENT CODE
          cpi     A,0xA0        ; GOTORW  GOTO?
; PC no branch -87
          brne    PC+2
          rjmp    GOTO3
          cpi     A,0xA1        ; GOSURW  GOSUB?
          breq    GOSU0
          cpi     A,0xBA        ; ERSRW  ERRSET
; PC no branch -70
          brne    PC+2
          rjmp    SES0
          cpi     A,0xA6        ; RESTRW  RESTORE?
; PC no branch 519
          brne    PC+2
          rjmp    REST0
          cpi     A,0xBE        ; EXITRW  EXIT?
; PC no branch 7589
          breq    PC+3
          jmp    BSERR         ; **  NOT ANY  **
          push    YL
          push    YH
          rjmp    EXIT0
;*
;*
;*     EXIT STATEMENT
;*
;*   SAME AS GOTO EXCEPT TERMINATES CURRENT FOR/NEXT
;*
; PC no jump 5160
EXIT:     call    DIRT
; PC no jump 5313
          call    LNUM          ; ADDR OF NEW LINE NUM
          push    ZL
          push    ZH            ; IN HL, SAVE IT
EXIT0:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
;*
EXIT1:    adiw    ZL,1
          ld      A,Z
          cpi     A,1           ; FRTYPE  CHECK FOR FOR LOOP ENTRY ON CONTROL STACK
          brne    EXIT2         ; IF NONE, THEN DO A GOTO
          ldi     YL,low(FORSZ-1)
          ldi     YH,high(FORSZ-1); 
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          rjmp    EXIT1         ; % extra line of code?
;*
;*  NOW GOTO THE LINE (TXA IN TOS)
EXIT2:    pop     YH
          pop     YL
          rjmp    GOTO3
;*
;*
;*  GOSUB STATEMENT
;*
; PC no jump 5129
GOSUB:    call    DIRT
; PC no jump 5282
          call    LNUM          ; WHERE TO GO IN HL (LINE NUMBER)
; PC no branch 7545
          brcc    PC+3
          jmp    BSERR
          _XCHG                 ; LINE NUMBER TO DE
; PC no jump 7910
GOSU0:    call    JOE           ; SKIP TO NEXT LINE, ALSO...
          mov     XH,ZH         ; JOE GIVES US THE RETURN ADDRESS IN HL
          mov     XL,ZL
          push    YL
          push    YH            ; SAVE LN
          ldi     YL,low(-3)
          ldi     YH,high(-3)
; PC no jump 8149
          call    PSHCS
          pop     YH
          pop     YL            ; % extra line of code
          st      Z,XH          ; STACK RETURN ADDRESS
          sbiw    ZL,1
          st      Z,XL
          sbiw    ZL,1
          ldi     c_tmp,2
          st      Z,c_tmp       ; GTYPE  MAKE CONTROL STACK ENTRY TYPE 'GOSUB'
; POP D  GET LN   % in revision
          rjmp    GOTO3         ; LINE NUM IS IN DE
;*
;*
;*
;*
;*     RETURN STATEMENT
;*
;*
;*
; PC no jump 5097
RETRN:    call    DIRT
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          adiw    ZL,1
;*
RET1:     ld      A,Z
          cpi     A,0           ; ETYPE  CHECK FOR STACK EMPTY
; PC no branch 1065
          brne    PC+2
          rjmp    CSERR
          cpi     A,2           ; GTYPE  CHECK FOR GOSUB TYPE
          breq    RET2
          cpi     A,5           ; SBTYPE  STACK BLOCK TYPE (INDICATING A FUNCTION CALL)
          breq    RET3
;*
;*  MUST HAVE BEEN FOR TYPE ENTRY, REMOVE IT
          ldi     YL,low(0x0011)
          ldi     YH,high(0x0011); FORSZ
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          rjmp    RET1
;*
;*  FOUND A GTYPE STACK ENTRY
RET2:     adiw    ZL,1
          ld      YL,Z          ; LOW ORDER RETURN TEXT ADDRESS
          adiw    ZL,1
          ld      YH,Z          ; HIGH ORDER RETURN TXT ADDRESS
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          ld      A,Y
          dec     A
; PC no branch -389
          breq    PC+2
          rjmp    NEXT4
          rjmp    END
;*
;STACK BLOCK ENTRY FOUND ON STACK, REMOVE IT THEN SEE WHAT
;*
RET3:     adiw    ZL,1          ; ADVANCE POINTER TO IFTERM
          ld      A,Z
          mov     XH,A          ; SAVE IT IN B
;*
          ldi     c_tmp,0xEB        ; ADI
          add     A,c_tmp       ; CMNDSP-SPCMND  DOES AN 8 BIT ADD
          MOV     YL,A
          ldi     A,')'         ; CMNDSP/256
          ldi     c_tmp,0xFF     ; ACI
          adc     A,c_tmp       ; 377Q  COMPLETE THE DBLE PRECISION ADD
          mov     YH,A          ; DE NOW HAS ADDRESS OF TOP OF NEW (I.E. OLD) 8080
;*
          _XCHG
          out     SPL,ZL        ;SPHL
          out     SPH,ZH        ; SET THE STACK PTR (WE HAVEN'T PUT THE STACK BACK YET)
; PUSH B  SAVE IFTERM (IN C)  % line added in revision
;*
RET31:    adiw    YL,1          ; COPY STACK BLOCK TO STACK
          ld      A,Y
          st      Z,A
          adiw    ZL,1
          inc     XH
          brne    RET31
;*
          _XCHG
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; WE ARE DONE WITH THE STACK BLOCK
;*
          adiw    ZL,1          ; ADVANCE PTR TO TYPE OF FUNCTION STACK ENTRY
          ld      A,Z
          cpi     A,0x7F        ; FNTYPE
          breq    RET4          ; JMP IF NUMERIC FUNCTION TYPE
;*
;*  STRING TYPE FUNCTION ENTRY FOUND ON STACK
;*
          push    ZL
          push    ZH            ; REMEMBER TSTKA FOR RETURNING RESULT STRING
; PC no jump 2712
          call    SEXPG
          pop     YH
          pop     YL
          push    YL
          push    YH
          push    ZL
          push    ZH
; PC no jump 7804
          call    JOE
          rcall   POPRG         ; POP THE ARGS
          pop     XH
          pop     XL
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          pop     YH
          pop     YL
          sbiw    YL,1
          push    XL
          push    XH
          rcall   RMOV          ; MOV THE RETURN VALUE 'DOWN' TO TOP OF STACK
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          pop     YH
          pop     YL            ; SIZE MUST BE RETURNED IN DE
          ret                   ; % extra line not in revision
;% code added in revision
;*
;*  RESTORE IFTERM AND RETURN FROM FUNC TO ITS CALLER
;*
;RETRET POP B  GET IFTERM
; MOV A,C
; STA IFTERM  RESTORE IFTERM
; RET .  THIS SHOULD TAKE US BACK TO FUNC
;%
;*
;*  FOUND A NUMERIC FUNCTION TYPE ENTRY
;*
RET4:     sbiw    ZL,1
          push    ZL
          push    ZH            ; PTR TO RESULT VALUE
          adiw    ZL,1
          push    ZL
          push    ZH            ; TSTKA PTR TO FNTYPE ENTRY
; PC no jump 2247
          call    EXPRB
; PC no jump 7776
          call    JOE
          pop     YH
          pop     YL
          rcall   POPRG
          pop     ZH
          pop     ZL
; PC no jump 7982
          call    PSHAS
          ret                   ; % extra line not in revision
; JMP RETRET  % code added in revision
;*
;*  SUBROUTINE ONLY USED BY RET, FOR POPPING OFF AND RESTORING
;*  ORIGINAL VALUES FOR THE FORMALS TO FUNCTION CALL
;*  ALSO RESTORES TXA TO CALLING ENVIRONMENT AND RETURNS
;*  TSTKA IN HL
;*  EXPECTS PTR TO FUNCTION TYPE ENTRY IN DE
;*
POPRG:    _XCHG
          adiw    ZL,1
          ld      YL,Z
          adiw    ZL,1
          ld      YH,Z
          _XCHG
          sts     TXA,ZL
          sts     TXA+1,ZH
          _XCHG
;*
POPR1:    adiw    ZL,1          ;  MOVE TO NEXT TYPE BYTE
          ld      A,Z
          cpi     A,4           ; EFTYPE  LOOK FOR END OF FUNCTION MARKER
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          brne    PC+2          ; RZ
          ret
;*
;*  MUST HAVE BEEN NUMERIC ARG (NO NEED TO CHECK)
          adiw    ZL,1
          ld      YL,Z          ; LOW ORDER BYTE OF SYMTAB PTR
          adiw    ZL,1
          ld      YH,Z          ; HIGH ORDER BYTE OF SYMTAB PTR
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
; PC no jump 7974
          call    POPA1
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          rjmp    POPR1
;*
;*
;% end of file BSM#STM1
;% start of file BSM#STM2
;*
;*
;*     DATA AND REM STATEMENTS
;*
; PC no jump 4939
DATA:     call    DIRT          ; DATA STATEMENT ILLEGAL AS DIRECT
;*
DATA1:    eor     A,A
;*  FALL THROUGH TO NEXTS (NON REM ENTRY PT)
;*
;*     ADVANCE TXA TO TERMINATOR OF CURRENT STATEMENT
;*     IF ACC HAS REMRW, THEN IGNORE EOSRW
;*
NEXTS:    ldi     XH,13
          cpi     A,0xA7        ; REMRW
          breq    NEXS0
          ldi     XH,0x97       ; EOSRW
NEXS0:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
NEXS1:    ld      A,Z
          cpi     A,0x96        ; LNRW  SKIP LN'S
          brne    NEXS9
          in r25,SREG
		  adiw    ZL,3          ; LNRW
          out SREG,r25
;          adiw    ZL,1          ; LOW
 ;         adiw    ZL,1          ; HIGH
          rjmp    NEXS1
NEXS9:    cpi     A,13
          breq    NEXS2
          adiw    ZL,1
          cp      A,XH           ; swapped to protect flags
          brne    NEXS1
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
NEXS2:    sts     TXA,ZL
          sts     TXA+1,ZH
          ret
;*
REM:      ldi     A,0xA7        ; REMRW
          rjmp    NEXTS
;*
;*
;*     DIMENSION STATEMENT
;*
; PC no jump 7758
DIM:      call    ANAME
          brcs    DIM1          ; VECTOR CASE
;*
;*  STRING CASE
;*
; PC no jump 4745
          call    STLK
; PC no branch 7246
          brcs    PC+3
          jmp    DMERR
; PC no jump 8003
          call    EATLP
; PC no jump 4930
          call    PFIXE
; PC no branch 7237
          brne    PC+3
          jmp    DMERR
; PC no jump 7999
          call    EATRP
; PC no jump 7583
          call    DIMST
; PC no jump 8022
DIM0:     call    SCOMA
          brcc    PC+2          ; RC
          ret
          rjmp    DIM           ; SEE IF MORE TO DO
;*
;*  VECTOR CASE
;*
DIM1:     mov     A,XL
          ori     A,' '         ; MTYPE
          mov     XL,A
; PC no jump 4713
          call    STLK
; PC no branch 7214
          brcs    PC+3
          jmp    DMERR
; PC no jump 7971
          call    EATLP
          ldi     ZL,low(2)
          ldi     ZH,high(2)
; PC no jump 7910
          call    ASTAB         ; ALLOCATE SPACE OF MAX SIZE
          push    ZL
          push    ZH
; PC no jump 4891
          call    PFIXE
          push    YL
          push    YH
          push    YL
          push    YH
;*
;*  BEGIN LOOP WHICH GETS DIMENSIONS
;*
DIM2:     ldi     ZL,low(2)
          ldi     ZH,high(2)   ; ALLOCATE DIMENSION IN DIMENTION TABLE
; PC no jump 7898
          call    ASTAB
          pop     YH
          pop     YL            ; DIMENSION SIZE
          mov     A,YH
          or      A,YL
; PC no branch 7187
          brne    PC+3
          jmp    DMERR         ; CAN'T BE ZERO
; PC no jump 7545
          _DSTOR         ; STORE DIMENSION IN TABLE
; PC no jump 7975
          call    SCOMA
          brcs    DIM3          ; JMP IF NO MORE DIMENSIONS
; PC no jump 4865
          call    PFIXE         ; GET NEXT DIMENSION
          pop     XH
          pop     XL            ; ACCUMULATING SIZE IN ELEMENTS
          push    YL
          push    YH
; PC no jump 3934
          call    IMUL
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; DO THIS SO THE CURRENT INDEX SIZE WILL BE ON TOP A
          push    ZL
          push    ZH
          rjmp    DIM2
;*
;*  DONE COLLECTING DIMENSION
; PC no jump 7927
DIM3:     call    EATRP
          pop     YH
          pop     YL            ; ACCUMULATED SIZE
          pop     ZH
          pop     ZL            ; SYMTAB ADDR TO MAX SIZE FIELD
; PC no jump 7544
          call    DIMM
          rjmp    DIM0
;*
;*
;*     STOP STATEMENT
;*
; PC no jump 4787
STOP:     call    DIRT
          ldi     ZL,low(IL1)
          ldi     ZH,high(IL1)
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; SAVE CONTINUE ADDRESS, DESTROY RETURN LINK
          push    XL
          push    XH            ; SAVE REGISTORS
          push    YL
          push    YH
          push    ZL
          push    ZH
;*
STOP1:    ldi     A,1           ; THIS ENTRY POINT FROM PCHECK
          sts     CONTF,A
          sts     DIRF,A
;*
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          push    ZL
          push    ZH            ; SAVE TXA ON STACK
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          _DAD_SP
          sts     SPTR,ZL
          sts     SPTR+1,ZH
;*
; PC no jump 6155
          call    CCRLF
          ldi     ZL,low(STOPS*2)
          ldi     ZH,high(STOPS*2)
; PC no jump 8101
          call    pgm_PRNT
          ldi     ZL,low(STOP2)
          ldi     ZH,high(STOP2)
          sts     IBUF,ZL
          sts     IBUF+1,ZH     ; ADDR OF SECOND PART TO IBUF
; PC no jump 7215
          jmp     ERM1

;*     END STATEMENT
;*
; PC no jump 4734
END:      call    DIRT
END1:     eor     A,A
          sts     CONTF,A
; PC no jump -3705
          jmp     CMND0
;*
;*
;*     READ STATEMENT
;*     RTXA SHOULD POINT TO EITHER A ',' OR A CR
;*
; PC no jump 7868
READ:     call    GC            ; TEST FOR FILE READ
          cpi     A,'#'
; PC no branch 1126
          brne    PC+2
          rjmp    SFREAD
; PC no jump 7768
READ7:    call    XTXA
; PC no jump 7871
READ1:    call    SCOMA
          brcc    READ2         ; PROCESS INPUT VALUE
;*
;*  SCAN FOR NEXT DATA STATEMENT
          ldi     XL,low(0x9F9F)
          ldi     XH,high(0x9F9F); DATARW*256+DATARW
; PC no jump 7597
          call    LSTAT
;*
;*  PROCESS VALUE (GIVE ERROR IF CARRY IS SET)
;*
; PC no jump 7753
READ2:    call    XTXA          ; TXA TO PROGRAM AGAIN
; PC no branch 7038
          brcc    PC+3
          jmp    RDERR         ; NOTE THAT CARRY ONLY SET ON ERROR HERE
; PC no jump 4270
          call    VAR
; PC no jump 7744
          call    XTXA
          breq    READ5         ; STRING CASE
;*
;*  NUMERIC CASE
;*
          push    ZL
          push    ZH
; PC no jump 4608
          call    CONST
; PC no branch 7022
          brcc    PC+3
          jmp    RDERR         ; BAD CONSTANT
          pop     YH
          pop     YL
; PC no jump 7704
          call    POPA1         ; STORE VALUE IN VARIABLE
          rjmp    READ6
;*
;*  STRING CASE
;*
READ5:    ldi     A,'"'
; PC no jump 2575
          call    STASS
;*
; PC no jump 7719
READ6:    call    XTXA
; PC no jump 7822
          call    SCOMA         ; MORE TO DO?
          brcc    READ7
          ret
;*
;*
;*     RESTORE STATEMENT
;*
; PC no jump 4813
RESTOR:   call    LNUM
          _XCHG
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1     ; IN CASE NO LINE NUMBER SUPPLIED
REST0:    brcs    PC+3          ; CNC
          call   FNEQLN
          sbiw    ZL,1          ; POINT TO THE CR BEFORE SPECIFIED LINE
          sts     RTXA,ZL
          sts     RTXA+1,ZH
          ret
;*
;*
;*     PRINT STATEMENT
;*
; PC no jump 7338
PRINT:    call    CFD
; PC no jump 7783
          call    GC            ; TEST FOR FILE PRINT
          cpi     A,'#'
; PC no branch 1124
          brne    PC+3
          jmp    SFPRINT
;*
;*  IF IN KEYBOARD MODE THEN CRLF
;*
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          lds     A,DIRF
          or      A,A
          breq    PC+3          ; CNZ
          call   CRLF          ; NEWLINE
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
;*
;*  SPECIAL CASE WHERE FIRST THING IS AN EOS MARK
;*
          cpi     A,13
; PC no branch 6022
          brne    PC+3
          jmp    CRLF
          cpi     A,0x97        ; EOSRW
; PC no branch 6017
          brne    PC+3
          jmp    CRLF
          ldi     ZL,low(IFTERM)
          ldi     ZH,high(IFTERM)
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 6010
          brne    PC+3
          jmp    CRLF
;*
; PC no jump 7749
PR0:      call    GC
          cpi     A,13
          brne    PC+2          ; RZ
          ret                   ; END OF STATEMENT
;*
          cpi     A,0x97        ; EOSRW
          brne    PC+2          ; RZ
          ret                   ; END OF STATEMENT
;*
          cpi     A,','
          breq    PR1
;*
          cpi     A,';'
          breq    PR1
          ldi     ZL,low(IFTERM)
          ldi     ZH,high(IFTERM)
          ld      c_tmp,Z
          cp      A,c_tmp
          brne    PC+2          ; RZ
          ret                   ; END OF STATEMENT
;*
          cpi     A,0x85        ; TABRW
; PC no branch 291
          brne    PC+2
          rjmp    PTAB          ; GO DO TABULATION
;*
          cpi     A,'%'
; PC no branch 95
          brne    PC+2
          rjmp    FORMAT        ; GO DO SET FORMAT
;*
; PC no jump 7208
          call    STEST
          breq    PSTR          ; GO PRINT A STRING
;*
          rcall   EXPRB         ; MUST BE EXPRESSION TO PRINT
; PC no jump 7584
          call    POPFP         ; POP VALUE TO FPSINK
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; POINTS TO WHERE THE VALUE WAS ON TOP OF STACK
; PC no jump 4841
          call    FPOUT         ; GO PRINT THE NUMERIC RESULT
;*
;*  CHECK LINE WILL OVERFLOW, LENGTH OF OUTPUT VALUE STRING IN
;*  REG B
;*
          lds     A,PHEAD
          add     A,XH
          mov     XH,A
          lds     A,LINLEN
          cp      A,XH
          brcc    PC+3          ; CC
          call   CRLF
; PC no jump 7890
          call    PRNT          ; PRINT TO '"' (STRING POINTER IN HL)
; PC no jump 5980
          call    SPDCK         ; PCHECK
;*
PR1:      ldi     A,';'
; PC no jump 7693
          call    SCANC
; PC no branch -78
          brne    PC+2
          rjmp    PR0           ; MORE ITEMS TO PROCESS (OR END OF LIST)
          ldi     A,','
; PC no jump 7685
          call    SCANC
; PC no branch 210
          brne    PC+2
          rjmp    CTAB
; PC no jump 5921
          jmp     CRLF          ; NO MORE ITEMS, CRLF AND RETURN
;*
PSTR:     lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; REMEMBER BASE ADDRESS OF STRING WE ARE ABO
          push    ZL
          push    ZH
; PC no jump 2205
          call    SEXPG
          pop     YH
          pop     YL            ; BASE ADDRESS OF STRING (COUNT IS IN HL)
          _XCHG
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
;*
PSTR1:    mov     A,YH
          or      A,YL
          breq    PR1
          lds     A,LINLEN
          mov     XH,A
          lds     A,PHEAD
          cp      A,XH
          brcs    PC+3          ; CNC
          call   CRLF
          ld      XH,Z
; PC no jump 5829
          call    OF1           ; THIS WILL PRINT CTRL-CHARACTERS IN NON-EXPANDED FORM
; PC no jump 5926
          call    SPDCK         ; PCHECK
          sbiw    ZL,1
          sbiw    YL,1
          rjmp    PSTR1
;*
;*
;*     FORMAT PROCESSOR
;*
; PC no jump 7185
FORMAT:   call    CFF
;*
; PC no jump 7622
PFRM1:    call    GCI
; PC no jump 4591
          call    INTGER
          brcc    PFRM2         ; JUMP IF FOUND WIDTH
          cpi     A,'D'
          breq    PFTWO
          ldi     ZL,low(COPT)
          ldi     ZH,high(COPT)
          ldi     XH,0x80       ; 200Q
          cpi     A,'Z'
          breq    PFONE
          ldi     XH,'@'        ; 100Q
          cpi     A,'C'
          breq    PFONE
          ldi     XH,' '        ; 40Q
          cpi     A,0xCB        ; PLSRW
          breq    PFONE
          ldi     XH,1
          cpi     A,'$'
          breq    PFONE
          ldi     XH,2
          cpi     A,'#'
; PC no branch 95
          breq    PC+2
          rjmp    PFRM3
;*
PFONE:    ld      A,Z
          or      A,XH
          st      Z,A
          rjmp    PFRM1
;*
; PC no jump 7111
PFTWO:    call    CFD
          rjmp    PFRM1
;*
;*  INTEGER WIDTH IN DE
PFRM2:    mov     A,ZH
          or      A,A
; PC no branch 6795
          breq    PC+3
          jmp    FMERR
          mov     A,ZL
          or      A,A
; PC no branch 6790
          brne    PC+3
          jmp    FMERR
          cpi     A,27          ; WMAX+1
; PC no branch 6785
          brcs    PC+3
          jmp    FMERR
          sts     CWIDTH,A
;*
;*  GET TYPE OF FORMAT
; PC no jump 7540
          call    GCI
          cpi     A,'I'
          breq    PF3
          cpi     A,'E'
          breq    PF4
          cpi     A,'F'
; PC no branch 6764
          breq    PC+3
          jmp    FMERR
;*
;*  GET CFRACT
PF4:      push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
; PC no jump 4493
          call    INTGER
; PC no branch 6757
          brcc    PC+3
          jmp    FMERR
          mov     A,ZH
          or      A,A
; PC no branch 6752
          breq    PC+3
          jmp    FMERR
          mov     A,ZL
          ldi     ZL,low(CWIDTH)
          ldi     ZH,high(CWIDTH)
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 6744
          brcs    PC+3
          jmp    FMERR
          sts     CFRACT,A
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          cpi     A,'E'
          brne    PF3           ; IF NOT E THEN SKIP THIS TEST
;*
          lds     A,CFRACT
          mov     XH,A
          lds     A,CWIDTH
          sub     A,XH
          cpi     A,6           ; 5+1
; PC no branch 6722
          brcc    PC+3
          jmp    FMERR
          ldi     A,'E'
;*
PF3:      sts     CFORM,A
;*
PFRM3:    ldi     ZL,low(COPT)
          ldi     ZH,high(COPT)
          ld      XH,Z
          ld      A,Z
          andi    A,0xFD        ; 375Q
          st      Z,A
          mov     A,XH
          andi    A,2
          breq    PC+3          ; CNZ
          call    DFC
          jmp     PR1
;*
CTAB:     lds     A,TWIDTH      ; TWIDTH  GET TAB FIELD WIDTH
          ldi     YL,0
          mov     XH,A
          lds     A,PHEAD       ; PRINTER HEAD POSITION
CT0:      mov     YH,A
          mov     A,YL
          add     A,XH          ; NEW POSITION=NEW POSITION+FIELD WIDTH
          MOV     YL,A
          mov     A,YH
          sub     A,XH          ; OLD POSITION=OLD POSITION-FIELD WIDTH
          brcc    CT0           ; DONE IF UNDERFLOW
          rcall   PTAB1         ; TAB TO NEW POSITION IN E
          rjmp    PR0
; PC no jump 7434
PTAB:     call    GCI           ; GOBBLE TAB RW
; PC no jump 7406
          call    EATLP
; PC no jump 4333
          call    PFIXE
; PC no jump 7405
          call    EATRP
          mov     A,YH          ; CHECK MAGNITUDE BETWEEN 0 AND 255 INCLUSIVE
          or      A,A
; PC no branch 6647
          breq    PC+3
          jmp    OBERR
          rcall   PTAB1
          rjmp    PR1
;*
PTAB1:    lds     A,PHEAD
          cp      A,YL
          brcs    PC+2          ; RNC
          ret
          ldi     XH,' '
; PC no jump 5631
          call    CHOUT
; PC no jump 5694
          call    SPDCK
          rjmp    PTAB1
;*
;*
;*     INPUT STATEMENT
;*
INPUT:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          push    ZL
          push    ZH            ; SAVE TXA
;*
; PC no jump 7400
          call    SCOMA
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; REMEMBER WHETHER OR NOT TO ECHO CARRIAGE RETTUR
;*
          ldi     A,0xC8        ; LPARRW  TEST FOR LIMIT OPTION
; PC no jump 7396
          call    SCANC
          brcs    PC+2          ; CNC
          rcall   IN10
;*
          ldi     A,'"'
; PC no jump 7388
          call    SCANC
; PC no branch 107
          brcs    PC+2
          rjmp    IN9           ; PRINT INITIAL QUOTED STRING
;*
INP0:     ldi     XH,'?'
; PC no jump 5596
          call    CHOUT
;*
; PC no jump 5831
INP1:     call    INLINE
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          breq    PC+3          ; CNZ
          call    CRLF
          ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1); INPUT ARRIVES AT IBUF1
;*
IN1:      push    YL
          push    YH            ; SAVE FOR FPIN
; PC no jump 3780
          call    VAR
          breq    IN5           ; STRING CASE
          pop     YH
          pop     YL
IN1B:     ld      A,Y           ; EAT LEADING SPACES
          cpi     A,' '
          brne    IN1A
          adiw    YL,1
          rjmp    IN1B
; PC no jump 5124
IN1A:     call    FPIN          ; INPUT FP NUMBER
          brcs    IN8
;*
; PC no jump 7340
IN4:      call    SCOMA
          brcs    IN7           ; JMP IF NO MORE TO DO
          mov     A,XH          ; GET THE TERMINATOR TO A
          cpi     A,','
          breq    IN1           ; GET THE NEXT INPUT VALUE FROM STRING
          cpi     A,13
          brne    IN8
;*
;*  GET NEW LINE  FROM USER
          ldi     XH,'?'
; PC no jump 5539
          call    CHOUT
          rjmp    INP0
;*
;*  STRING CASE
; PC no jump 6926
IN5:      call    FTXA          ; SWITCH TOP OF STACK WITH TXA
          ldi     A,13
; PC no jump 2057
          call    STASS         ; ASS STRING TO VARIABLE FROM INPUT LINE
          pop     ZH
          pop     ZL
          sts     TXA,ZL
          sts     TXA+1,ZH      ; RESTORE TXA
          ldi     XH,13
          rjmp    IN4
;*
;*  NO MORE INPUT DESIRED
IN7:      mov     A,XH
          cpi     A,13
          pop     YH
          pop     YL            ; POP THE "CR FLAG"
          pop     ZH
          pop     ZL            ; ORIGINAL TXA
          brne    PC+2          ; RZ
          ret
          push    ZL
          push    ZH            ; RESTORE SAVED TXA TO STACK
          push    YL
          push    YH            ; RESTORE CRFLAG
;% revised code
;*
;*  INPUT ERROR, START OVER
;*
;IN8 LHLD ERRLN  TEST FOR ERROR TRAPPING
; MOV A,H
; ORA L
; JNZ INERR  IF SO THEN INPUT ERROR
;*
;*  HANDEL THE ERROR LOCALY
;*
IN8:      pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          breq    PC+3          ; CNZ
          call    CRLF          ; NO CRLF IF IN NOCRLF MODE
          ldi     ZL,low(INSTR)
          ldi     ZH,high(INSTR)
; PC no jump 7474
          call    PRNT
          pop     ZH
          pop     ZL            ; POP ORIGINAL TXA OF PRINT STATEMENT
          sts     TXA,ZL
          sts     TXA+1,ZH      ; RESTORE ORIGINAL TXA
          rjmp    INPUT
;*
;*  PRINT INITIAL QUOTED STRING (TXA IN HL)
IN9:      ld      XH,Z
          mov     A,XH
          adiw    ZL,1
          cpi     A,'"'
          breq    IN9A
; PC no jump 5449
          call    OF1
          rjmp    IN9
;% revised code above checked for cr and reported BS error
IN9A:     sts     TXA,ZL
          sts     TXA+1,ZH
; PC no jump 5540
          call    SPDCK
          ldi     XH,','
; PC no jump 7224
          call    EATC
          rjmp    INP1
;*
; PC no jump 4087
IN10:     call    DIRT
; PC no jump 4138
          call    PFIXE         ; GET COUNT LIMIT EXPRESSION
          mov     A,YH
          or      A,A
; PC no branch 6455
          breq    PC+3
          jmp    OBERR         ; TOO BIG
          mov     A,YL
          cpi     A,LINMAX+1   ; 0x85 LINMAX+1  TOO BIG?
; PC no branch 6449
          brcs    PC+3
          jmp    OBERR         ; YES
          sts     ICNT,A        ; INPUT COUNT LIMIT
;*
          ldi     XH,','
; PC no jump 7196
          call    EATC
;*
; PC no jump 4116
          call    PFIXE         ; GET TIME LIMIT EXPRESSION
          _XCHG
          sts     ITIM,ZL
          sts     ITIM+1,ZH     ; SAVE IT
;*
; PC no jump 7184
          jmp     EATRP         ; EAT RP AND RETURN
;*
;*
;*
;*     DEF STATEMENT
;*
; PC no jump 4023
DEF:      call    DIRT0
; PC no jump 6942
          call    FEND
          rjmp    DATA
;*
; PC no jump 4014
FN:       call    DIRT0
CSERR:    ldi     XL,low(0x4353)
          ldi     XH,high(0x4353)
; PC no jump 6441
          jmp     ERROR         ; CAN'T BE A STATEMENT
;*
;*
;*     OUT <PORT>,<VALUE>
;*
;% this code somewhat revised in PTDOS
;irrelevant as AVR works different
; PC no jump 4067
L_OUT:    call    PFIXE         ; <PORT>
          ldi     A,0xD3        ; 323Q  'OUT' OP CODE
          rcall   OSET          ; SETUP
; PC no jump 7163
          call    SCOMA
; PC no jump 4056
          call    PFIXE         ; <VALUE>
; PC no jump 5914
          jmp     OBUF
;*
;*     OBUF SETUP ROUTINE
;*
OSET:     ldi     ZL,low(OBUF)
          ldi     ZH,high(OBUF)
          st      Z,A           ; OPCODE
          mov     A,YH          ; HIGH ORDER BITS SHOULD BE 0
          or      A,A
; PC no branch 6366
          breq    PC+3
          jmp    OBERR
          adiw    ZL,1
          st      Z,YL          ; DEVICE CODE
          adiw    ZL,1
          ldi     c_tmp,0xC9
          st      Z,c_tmp       ; 311Q  RETURN INSTRUCTION
          ret
;% Poke
;*
;*
;*     FILL A BYTE  FILL(ADDRESS,BYTE)
;*
; PC no jump 4035
FILL:     call    PFIXE
          push    YL
          push    YH            ; MEMORY ADDRESS
; PC no jump 7135
          call    SCOMA
; PC no jump 4028
          call    PFIXE
          pop     ZH
          pop     ZL
          st      Z,YL
          ret
;*
;*
;*     WAIT FOR SOME BITS IN AN I/O PORT
;*
; PC no jump 3968
SWAIT:    call    DIRT
; PC no jump 4019
          call    PFIXE         ; GET PORT
          ldi     A,0xDB        ; A 'IN' INST
          rcall   OSET
;*
          ldi     XH,','
; PC no jump 7086
          call    EATC
;*
; PC no jump 4006
          call    PFIXE         ; GET MASK
          mov     A,YH
          or      A,A
; PC no branch 6323
          breq    PC+3
          jmp    OBERR
          push    YL
          push    YH
;*
          ldi     XH,','
; PC no jump 7072
          call    EATC
;*
; PC no jump 3992
          call    PFIXE         ; GET WORD
          mov     A,YH
          or      A,A
; PC no branch 6309
          breq    PC+3
          jmp    OBERR
          push    YL
          push    YH
;*
; PC no jump 5844
SW0:      call    OBUF          ; CALL THE IN INST
          pop     XH
          pop     XL
          pop     YH
          pop     YL
          and     A,YL          ; MASK
          cp      A,XL          ; AND TEST
          brne    PC+2          ; RZ
          ret                   ; BIT FOUND!
          push    YL
          push    YH
          push    XL
          push    XH
; PC no jump 5331
          call    PCHECK        ; IN CASE OF A LONG WAIT
          rjmp    SW0           ; WAIT FOR IT
;*
;*
;*
;*     SET CURSOR POSITION
;*
;% revised code check for VDM port
;SCURS LDA XOPORT
;ORA A
;JNZ NAERR
;*
; PC no jump 7071
SCURS:    call    SCOMA
          brcc    SCUR0         ; TAKE DEFAULT FOR Y
; PC no jump 7051
          call    GC            ; DEFAULTS?
          cpi     A,13
          breq    SCUR1         ; FOR X AND Y
          cpi     A,0x97        ; EOSRW
          breq    SCUR1         ; FOR Y AND X
; PC no jump 3948
          call    PFIXE
          mov     A,YL          ; RESULT MOD 256
          sts     LY,A          ; SET LAST Y POS
;*
; PC no jump 7045
          call    SCOMA
          brcs    SCUR1         ; TAKE DEFAULT
; PC no jump 3935
SCUR0:    call    PFIXE
          mov     A,YL          ; RESULT MOD 256
          sts     LX,A          ; SET LAST X POS
;*
;IF SOLOS
SCUR1:    lds     A,LX
          andi    A,'?'         ; 64-1
          sts     PHEAD,A
          ldi     XH,1          ; THE X ...
; PC no jump -2225
          call    ESCSEQ        ; ESCAPE SEQUENCE
;ENDF
;*
          lds     A,LY
;*
;IF SOLOS
          ldi     XH,2          ; THE Y ...
; PC no jump -2233
          call    ESCSEQ        ; ESCAPE SEQUENCE
;ENDF
;*
          ret
;*
;*

;*
;*
;*     SEARCH STATEMENT
;*
SSEAR:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; SAVE LEFT HAND STRING ADDRESS
          sts     LHSBA,ZL
          sts     LHSBA+1,ZH
          rcall   SEXPG         ; GET STRING TO STACK
          push    ZL
          push    ZH            ; SAVE LHS SIZE
;*
          ldi     XH,','        ; SYNTAX
; PC no jump 6969
          call    EATC
;*
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; SAVE RHS BA
          sts     RHSBA,ZL
          sts     RHSBA+1,ZH
          rcall   SEXPG
          push    ZL
          push    ZH            ; RHSSZ
;*
          ldi     XH,','
; PC no jump 6954
          call    EATC
;*
; PC no jump 3395
          call    VAR           ; GET INDEX VAR ADDR INTO...
; PC no branch 6199
          brne    PC+3
          jmp    TYERR
          sts     XC,ZL
          sts     XC+1,ZH
          rcall   ZEX           ; INITIALIZE IT TO ZERO (NOT FOUND)
;*
          lds     ZL,LHSBA     ;LHLD
          lds     ZH,LHSBA+1    ; POP THE STACK
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
;*
          pop     ZH
          pop     ZL            ; RHSSZ
          pop     YH
          pop     YL            ; LSHSZ
; PC no jump 3811
          _DSUB          ; LH:=HL (RHSSZ) - DE (LHSSZ)
          adiw    ZL,1
          sts     XA,ZL
          sts     XA+1,ZH       ; :=NUMBER OF TIMES TO COMPARE
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); INIT INDEX
          sts     XB,ZL
          sts     XB+1,ZH
          mov     XH,YH         ; DE (LHSSZ) TO BC
          mov     XL,YL
          brcc    PC+2          ; RC
          ret                   ; FLAGS FROM CALL TO DSUB ABOVE (LHS>RHS?)
;*
SSEA0:    push    XL
          push    XH            ; LHSSZ
          push    YL
          push    YH            ; RHSSZ
; PC no jump 7149
          call    SCOMP         ; COMPARE
          pop     YH
          pop     YL
          pop     XH
          pop     XL
          breq    SSEA1         ; EQU, SET INDEX AND RETURN
;*
          lds     ZL,XA     ;LHLD
          lds     ZH,XA+1       ; # OF TIMES
          sbiw    ZL,1          ; LESS 1
          sts     XA,ZL
          sts     XA+1,ZH
          mov     A,ZH          ; TEST FOR DONE
          or      A,ZL
          brne    PC+2          ; RZ
          ret                   ; NO MATCH
;*
          lds     ZL,XB     ;LHLD
          lds     ZH,XB+1       ; INDEX:=INDEX+1
          adiw    ZL,1
          sts     XB,ZL
          sts     XB+1,ZH
;*
          lds     ZL,RHSBA     ;LHLD
          lds     ZH,RHSBA+1    ; MOVE UP ONE POSITION IN RHS
          sbiw    ZL,1
          sts     RHSBA,ZL
          sts     RHSBA+1,ZH
;*
          rjmp    SSEA0         ; NEXT!
;*
;*
SSEA1:    lds     ZL,XB     ;LHLD
          lds     ZH,XB+1       ; INDEX
          adiw    ZL,1
; PC no jump 3773
          call    FLOAT         ; TO ARG STACK
          lds     ZL,XC     ;LHLD
          lds     ZH,XC+1       ; ADDR OF VAR
; PC no jump 6762
          call    POPAS         ; POP ARG STACK TO VAR
          ret                   ; ALL DONE
;*
;*
;*     PAUSE
;*
; PC no jump 3733
SPAW:     call    DIRT
; PC no jump 3784
          call    PFIXE
          mov     A,YH
          or      A,YL
          brne    PC+2          ; RZ
          ret                   ; VERRY FUNNY !!!
;*
PAW0:     ldi     ZL,low(0x0322)
          ldi     ZH,high(0x0322); TIMCONST  GET TIMMING CONSTANT
PAW1:     sbiw    ZL,1          ; TICK...
; PC no jump 5132
          call    PCHECK        ; INCASE USER WANTS NOT TO WAIT
          mov     A,ZH
          or      A,ZL
          brne    PAW1
;*
          sbiw    YL,1          ; ...TOCK
          mov     A,YH
          or      A,YL
          brne    PAW0
          ret                   ; TIME'S UP, BYE...BYE...
;% end of file BSM#STM2
;% start of file BSM#STM3
;*
;*
;*  STRING FILL STATEMENT
;*
;% string fill does not seem to be in this version
;% end of file BSM#STM3
;*
;*
;*     THE  FILE  RELATED  STATMENTS
;*
;*
;% start of file BSM#FIL1
;*
;*     FILE STATEMENT
;*       FILE #<EXP>;<SEXP>,<EXP>[,<VAR>]
;*
;*
; PC no jump 3705
SFILE:    call    DIRT
          rcall   OPEFCB        ;  GET USER'S FILE ID #
          brne    PC+2          ; CZ
          rcall   FLCLZ         ; ALREADY OPEN, CLOSE IT (CLOBBERS CUFID)
; PC no branch 6066
          brcc    PC+3
          jmp    FDERR         ; NO MORE ROOM, ABORT
;*   
          ldi     XH,';'        ; SYNTAX
; PC no jump 6822
          call    EATC
;*   
          rcall   GFNX          ; GET FILE NAME EXPRESSION
;*   
          ldi     XH,','
; PC no jump 6814
          call    EATC
;*   
; PC no jump 3734
          call    PFIXE
          mov     A,YH
          or      A,A
; PC no branch 6051
          breq    PC+3
          jmp    OBERR
          ldi     A,'T'
          sts     CFT,A
          mov     A,YL
          rcall   S_156B
; PC no jump 6821
          call    SCOMA
; PC no branch 332
          brcc    PC+2
          rjmp    CLOFCB
; PC no jump 3232
          call    VAR
; PC no branch 6036
          brne    PC+3
          jmp    TYERR
;*   
          push    ZL
          push    ZH            ; SAVE VAR ADDR
          lds     A,CACRE
          mov     ZL,A          ; INTO HL
          ldi     ZH,0
; PC no jump 3674
          call    FLOAT
          pop     YH
          pop     YL            ; VAR ADDR
; PC no jump 6666
          call    POPA1
          rjmp    CLOFCB
;*
;*
;*
;*     CLOSE STASTEMENT
;*       CLOSE #<EXP>[,#<EXP>]...
;*
;*
; PC no jump 3634
SFCLOSE:  call    DIRT
XFCLOSE:  rcall   OPEFCB        ; GET CSFID
; PC no branch 5998
          breq    PC+3
          jmp    FDERR         ; FILE NOT DECLARED
          rcall   FLCLZ         ; CLOSE FILE
          rcall   CLOFCB        ; CLOSE FCB TO MAKE A HOLE IN THE TABLE
; PC no jump 6777
          call    SCOMA         ; SYNTAX
          brcc    PC+2          ; RC
          ret                   ; NO COMMA, DONE
          rjmp    SFCLOSE
;*
;*
;*
;*     REWIND STATEMENT
;*       REWIND #<EXP>[,#<EXP>]...
;*
;*
; PC no jump 3612
SFREWIND: call    DIRT
XFREWIND: rcall   OPEFCB        ; OPEN FCB
; PC no branch 5976
  ;        breq    PC+3
   ;       jmp    FDERR         ; NOT DEFINED
          rcall   FLREW         ; REWIND FILE
          rcall   CLOFCB        ; CLOSE FCB
; PC no jump 6755
          call    SCOMA         ; SYNTAX
          brcc    PC+2          ; RC
          ret                   ; NO COMMA, DONE
          rjmp    SFREWIND      ; XFREWIND
;*
;*
;*
;*     FILE READ STATEMENT
;*       READ #<EXP>;<VAR>[,<VAR>]...
;*
;*
; PC no jump 3590
SFREAD:   call    DIRT
          rcall   OPEFCB
; PC no branch 5954
          breq    PC+3
          jmp    FDERR         ; NOT DEFINED
;*
          ldi     XH,';'        ; SYNTAX
; PC no jump 6710
          call    EATC
;*
SFR0:     ldi     YL,low(IBUF)
          ldi     YH,high(IBUF)
          rcall   FLR1I         ; READ 1 ITEM
          breq    SFR8          ; CLOSE FCB AND EXIT TO NEXT STATEMENT ON LINE
;*
; PC no jump 3142
          call    VAR           ; WHERE DOES THE VALUE GO?
          push    ZL
          push    ZH            ; WHERE HL POINTS
;*
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; SAVE TXA AND POINT TO THE INPUT BUFFER
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          push    ZL
          push    ZH
          ldi     ZL,low(IBUF)
          ldi     ZH,high(IBUF)
          sts     TXA,ZL
          sts     TXA+1,ZH
;*
          breq    SFRST         ; DISPATCH ON Z SET BY VAR (Z=1 FOR STRING)
;*
; PC no jump 3472
          call    CONST         ; NUMERIC, CONVERT
          pop     YH
          pop     YL            ; DE POINT TO THE VAR
          brcs    PC+3          ; CNC
          call   POPA1         ; POP TO VAR IF NO TYPE ERROR
SFR1:     pop     ZH
          pop     ZL            ; GET SAVED TXA
          sts     TXA,ZL
          sts     TXA+1,ZH
; PC no branch 5920
          brcc    PC+3
          jmp    TYERR         ; C SET BY 'CONST' ABOVE OR 'FSRST' BELOW
; PC no jump 6693
          call    SCOMA         ; SYNTAX
          brcc    SFR0          ; MORE ITEMS
;*
SFRR:     rcall   CLOFCB        ; NO COMMA, CLOSE FCB
          rjmp    REM           ; RETURN AFTER SKIP
;*
SFR8:     rcall   CLOFCB
          rjmp    DATA1         ; SKIP REST OF READ STATEMENT
;*
SFRST:    ldi     A,13          ; STRING ASSIGN TO A CR
          pop     ZH
          pop     ZL            ; POINTER TO VAR
          rcall   STASS
          eor     A,A           ; CLEAR CARRY
          rjmp    SFR1
;*
;*
;*
;*     FILE PRINT STATEMENT
;*       PRINT #<EXP>;<EXP>[,<EXP>]...
;*
;*
; PC no jump 3507
SFPRINT:  call    DIRT
          rcall   OPEFCB        ; OPEN THE FCB
; PC no branch 5871
          breq    PC+3
          jmp    FDERR         ; NO FCB, FILE NOT DECLARED
;*
;% CALL PRS  PROCESS RANDOM/SERIAL SEEK/SPACE
;*
          ldi     XH,';'        ; SYNTAX
;*
; PC no jump 6627
          call    EATC
          lds     ZL,COPT     ;LHLD
          lds     ZH,COPT+1
          sts     TOPT,ZL
          sts     TOPT+1,ZH
          lds     ZL,CWIDTH     ;LHLD
          lds     ZH,CWIDTH+1
          sts     V_2618,ZL
          sts     V_2618+1,ZH
; PC no jump 6190
          call    CFF
;*
; PC no jump 6115
SFP0:     call    STEST         ; STRING OR NUMERIC?
          breq    SFPST         ; STRING
;*
          rcall   EXPRB         ; EVALUATE EXPRESSION
; PC no jump 6491
          call    POPFP         ; POP TO FPSINK
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; POINT TO RESULT
; PC no jump 3748
          call    FPOUT         ; CONVERT TO ASCII
;*
          mov     YL,XH         ; HAS COUNT FROM FPOUT
          ldi     YH,0
;*
L_1372:   ld      A,Z           ; % part of cassete file extentions
          cpi     A,' '
          brne    L_137D
          adiw    ZL,1
          sbiw    YL,1
          rjmp    L_1372
L_137D:   ldi     XL,low(0x0001)
          ldi     XH,high(0x0001)
;*
SFP1:     rcall   FLW1I         ; WRITE 1 ITEM
;% CALL CLOFCB  CLOSE FCB
; PC no jump 6598
SFP3:     call    SCOMA         ; SYNTAX
          brcc    SFP0          ; A COMMA, NEXT ITEM
;% RET .
;*
;*
;%SFP2 CALL FORMAT
;% JMP SFP3
          rcall   CLOFCB
          lds     ZL,TOPT     ;LHLD
          lds     ZH,TOPT+1
          sts     COPT,ZL
          sts     COPT+1,ZH
          lds     ZL,V_2618     ;LHLD
          lds     ZH,V_2618+1
          sts     CWIDTH,ZL
          sts     CWIDTH+1,ZH
          ret
;*
;*  ITEM IS A STRING EXPRESSION
;*
SFPST:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; POP STRING
          push    ZL
          push    ZH
          rcall   SEXPG         ; GET STRING EXPRESSION
          pop     YH
          pop     YL
          _XCHG                 ; DE HAS COUNT, HL HAS BASE ADDR
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
;*
          ldi     XL,low(-1)
          ldi     XH,high(-1)
          rjmp    SFP1          ; WRITE IT
;*
;*
;*
;*     OPEN FCB
;*       THE USER'S FILE ID NUMBER IS GOTTEN FROM THE
;*       PROGRAM TEXT.  THE OPEN FILE TABLE (OFT) IS SEARCHED FOR
;*       THE CORRISPONDING FCB.
;*       IF FOUND:
;*        THE ENTRY IS COPIED TO A STATIC
;*        MEMORY AREA FOR EASY ADDRESSING.
;*       IF NOT FOUND:
;*        A NULL ENTRY IS PREPARED FOR 'SFILE'
;*       ON RETURN: Z=1, C=0   FCB FOUND
;*                  Z=0, C=1   OFT IS FULL
;*                  Z=0, C=0   A NULL ENTRY WAS FOUND
;*                  Z=1, C=1   NOT USED
;*       'OFCB' POINTS TO THE OPENED ENTRY IN THE 'OFT'
;*
;*
OPEFCB:   ldi     XH,'#'        ; SYNTAX
; PC no jump 6529
          call    EATC
; PC no jump 3449
          call    PFIXE         ; GET USER'S FILE ID INTO DE
OPEEOF:   mov     A,YH
          or      A,A           ; TOO BIG?
; PC no branch 5766
          breq    PC+3
          jmp    OBERR         ; YES
          or      A,YL          ; TOO SMALL?  (NOTE: MOVE E TO A AND TEST)
; PC no branch 5762
          brne    PC+3
          jmp    OBERR         ; TOO SMALL
; PC no branch 5759
          brpl    PC+3
          jmp    OBERR         ; TOO BIG
;*  AHHHH JUST RIGHT
;*
          sts     CUFID,A       ; IN CASE OF NON-MATCH, PLACE UFID IN COFCB
;*
          ldi     ZL,low(OFTE)
          ldi     ZH,high(OFTE) ; INIT OFCB TO OFTE TO SHOW NO NULL ENTRIES FOUND
          sts     OFCB,ZL
          sts     OFCB+1,ZH
          ldi     ZL,low(OFT+OFTEZ)
          ldi     ZH,high(OFT+OFTEZ); ADDR OF 2ND ENTRY (FIRST IS USED BY FILE CMDS)
          ldi     XL,low(OFTEZ)
          ldi     XH,high(OFTEZ); SIZE OF AN ENTRY
;*
OPE0:     ld      A,Z           ; TAKE A TRIP THRU THE TABLE
          cp      A,YL
          breq    OPE3          ; MATCH FOUND
          cpi     A,0xFF        ; EOT?
          breq    OPE2          ; END OF TABLE
          or      A,A           ; HOLE?
          brne    OPE1          ; NO
          sts     OFCB,ZL
          sts     OFCB+1,ZH     ; IN CASE A MATCH IS NOT FOUND, POINT TO A NULL ENT
OPE1:     add     ZL,XL     ; DAD B
          adc     ZH,XH         ; POINT TO NEXT ENTRY
          rjmp    OPE0
;*
;*  NO MATCH
;*
OPE2:     lds     ZL,OFCB     ;LHLD
          lds     ZH,OFCB+1     ; WAS A HOLE FOUND?
          ld      A,Z
          or      A,A
          sec
          breq    PC+2          ; RNZ
          ret                   ; TABLE FULL  C=1, Z=0
          dec     A             ; MAKE Z=0
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          ret                   ; THERE IS A HOLE AND HERE IT IS!  C=0, Z=0
;*
;*  MATCH FOUND
;*  NOTE:  ENTRY SIZE IS IN C
;*
OPE3:     sts     OFCB,ZL
          sts     OFCB+1,ZH     ; SAVE POINTER TO MATCH
          ldi     YL,low(CUFID)
          ldi     YH,high(CUFID); ADDR OF CURRENT FCB (STATIC BLOCK)
; PC no jump 5921
          jmp     COPY          ; FOUND MATCH  C=0, Z=1
;*
;*
;*     CLOSE FCB
;*       WRITE THE CURRENT FCB WHERE OFCB OPINTS
;*       (I.E.  UPDATE DYNAMIC TABLE FROM STATIC BLOCK)
;*
CLOFCB:   ldi     XL,20         ; C GETS ENTRY SIZE
          lds     ZL,OFCB     ;LHLD
          lds     ZH,OFCB+1
          _XCHG                 ; DE GETS ENTRY IN TABLE ADDRESS
          ldi     ZL,low(CUFID)
          ldi     ZH,high(CUFID); HL GETS ADDR OF CURRENT OPEN FCB
; PC no jump 5909
          jmp     COPY          ; CLOSE; COPY SETS Z!
;*
;*
;*
;*     GET FILE NAME EXPRESSION
;*       SETS CFN IN THE CURRENT OPEN FCB
;*       SETS CSFID TO THE SPECIFIED SYSTEM LEVEL
;*            FILE # (DEFAULT = 1 FOR CASSETT STUFF)
;*
;*
GFNX:     ldi     A,1
          sts     CSFID,A
;*
          lds     A,DIRF
          or      A,A
; PC no branch 75
          breq    PC+2
          rjmp    GFN3          ; KEYBOARD MODE
;*
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; GET FILE NAME STRING EXPRESSION
          push    ZL
          push    ZH            ; SAVE BASE ADDR OF STRING
          rcall   SEXPG
          pop     YH
          pop     YL
          _XCHG
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; SAVE BASE ADDR OF STRING
;*
          mov     A,YH          ; TEST LENGTH
          or      A,A
; PC no branch 5709
          breq    PC+3
          jmp    BSERR         ; TOO LONG FOR SURE
          or      A,YL
; PC no branch 5705
          brne    PC+3
          jmp    BSERR         ; TOO SHORT
;*
          ldi     YH,6          ; FNSIZ+1  FOR LENGTH TEST
          ldi     XL,low(CFN)
          ldi     XH,high(CFN)  ; POINTER TO FILE NAME IN CURRENT FCB
;*
GFN0:     ld      A,Z
          sbiw    ZL,1
;*
          cpi     A,'/'
          breq    S_1447
;*
          dec     YH            ; LENGTH TEST
; PC no branch 5689
          brne    PC+3
          jmp    BSERR         ; TOO LONG, SORRY
          st      X,A
          adiw    XL,1
          dec     YL            ; END OF STRING?
          brne    GFN0          ; NO, CONTINUE
;*
GFN05:    mov     A,YH
          cpi     A,6           ; FNSIZ+1
; PC no branch 5677
          brne    PC+3
          jmp    BSERR         ; EMPTY FILE NAME!!
          eor     A,A
GFN1:     st      X,A           ; ZERO OUT REMAINDER
          adiw    XL,1
          dec     YH
          brne    GFN1
          ret                   ; DONE, EXIT
;*
S_1447:   ld      A,Z           ; % cassete file system 
          subi    A,'1'           ;SUI
; PC no branch 5663
          brcc    PC+3
          jmp    BSERR
          cpi     A,2
; PC no branch 5658
          brcs    PC+3
          jmp    BSERR
          inc     A
          sts     CSFID,A
          rjmp    GFN05
;*
;*
;*     CODE FOR FILE NAMES IN COMMANDS
;*
GFN3:     lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ldi     YH,6          ; FNSIZ+1
          ldi     XL,low(CFN)
          ldi     XH,high(CFN)
;*
; PC no jump -4505
GFN4:     call    GF1           ; CALL Get Filter 1
          adiw    ZL,1
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; INCASE THERE'S A COMMA OR CR
          sbiw    ZL,1
          cpi     A,'/'
          breq    S_1447
          sbiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          adiw    ZL,1          ; INCASE THERE'S NEITHER (POINT TO NEXT CHAR)
;*
          cpi     A,','
; PC no branch -64
          brne    PC+2
          rjmp    GFN05
          cpi     A,13
; PC no branch -69
          brne    PC+2
          rjmp    GFN05
;*
          dec     YH
; PC no branch 5610
          brne    PC+3
          jmp    BSERR         ; TOO MANY
;*
          st      X,A
          adiw    XL,1
          rjmp    GFN4
;*
;*
;*
;*     FILE REWIND PRIMITIVE
;*
;*
FLREW:    lds     A,CSFID       ; REWIND THE FILE
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -5254
          call    cFCLOS
; PC no branch 5493
          brcc    PC+3
          jmp    CLERR
; PC no jump 4565
          call    CCRLF
          rcall   TTON
          ldi     ZL,low(PtUMsg*2)
          ldi     ZH,high(PtUMsg*2)
		  call    pgm_PRNT
          lds     XH,CSFID
          ldi     c_tmp,'0'        ; ADI
          add     XH,c_tmp       ; % Bias the tape recorder number for display 
          ;sts     PtUMsg+18,A
		  call   CHOUT
		  adiw ZL,1
; PC no jump 6500
          call    pgm_PRNT
          ldi     ZL,low(RwNDMsg*2)
          ldi     ZH,high(RwNDMsg*2)
; PC no jump 6494
          call    pgm_PRNT
; PC no jump 4670
          call    IF2           ; % input filter 2. wait for keypress
; PC no jump 4541
          call    CRLF
          rcall   TTOFF
          lds     A,CSFID
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
		  clc
; PC no jump -5304
          call    cFOPEN
; PC no branch 5440
          brcc    PC+3
          jmp    OPERR
          ldi     ZL,low(CEOF)
          ldi     ZH,high(CEOF)
          ldi     c_tmp,4
          st      Z,c_tmp       ; LAST WAS REWIND
          ret
;*
;*
;*     TAPE ON ROUTINE
;*
TTON:     lds     A,CSFID
          mov     XH,A
          ldi     A,3
          sub     A,XH
          lsr     A      ;RRC -- check cary rotate
          lsr     A      ;RRC -- check cary rotate
          ;OUT    0xFA ;-- adjust this with macro
          ret
;% end of file BSM#FIL1
;start of file BSM#FIL2
;*
;*    TAPE OFF ROUTINE
;*
TTOFF:    eor     A,A
          ;OUT    0xFA ;-- adjust this with macro
          ret
;*
;*
;*
;*     FILE WRITE ONE ITEM PRIMITIVE
;*
;*
FLW1I:    lds     A,CACRE       ; CHECK ACCESS RECEIVED
          andi    A,2           ; 2=WRITE ENABLE
; PC no branch 5404
          brne    PC+3
          jmp    ACERR         ; ACCESS ERROR
;*
          ldi     A,3           ; LAST WAS WRITE
          sts     CEOF,A
;*
L_14E5:   mov     A,YH          ; % part of cassete write system FLW1I
          or      A,YL
          breq    L_1503
          sbiw    YL,1
          push    YL
          push    YH
          push    XL
          push    XH
          ld      A,Z
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          mov     XH,A
          push    ZL
          push    ZH
          lds     A,CSFID
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -5354
          call    cWRBYT
; PC no branch 5399
          brcc    PC+3
          jmp    WTERR
          pop     ZH
          pop     ZL
          pop     XH
          pop     XL
          pop     YH
          pop     YL
          rjmp    L_14E5
;*
L_1503:   push    ZL
          push    ZH
          ldi     XH,13
          lds     A,CSFID
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -5375
          call    cWRBYT
; PC no branch 5378
          brcc    PC+3
          jmp    WTERR
          pop     ZH
          pop     ZL
          ret
;*
;*
;*
;*     FILE READ ONE ITEM PRIMITIVE
;*
;*
FLR1I:    lds     A,CACRE       ; CHECK ACCESS RECEIVED
          andi    A,1           ; 1=READ ENABLE
; PC no branch 5344
          brne    PC+3
          jmp    ACERR         ; ACCESS ERROR
;*
;*  TEST LAST ACCESS
          ldi     ZL,low(CEOF)
          ldi     ZH,high(CEOF)
          ld      A,Z
          cpi     A,3           ; FOR WRITE
; PC no branch 5335
          brne    PC+3
          jmp    ACERR         ; CAN'T READ AFTER A WRITE
          cpi     A,6           ; FOR EOF
; PC no branch 5330
          brne    PC+3
          jmp    ACERR         ; ATTEMPT TO READ PAST EOF
;*
;*  DO THE READ
;*
          ldi     c_tmp,2
          st      Z,c_tmp
          eor     A,A
          mov     XL,A
;*
L_152E:   push    XL
          push    XH
          push    YL
          push    YH
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
          lds     A,CSFID
; PC no jump -5420
          call    cRDBYT
          pop     YH
          pop     YL
          pop     XH
          pop     XL
          brcs    L_1548
; PC no jump -4708
          call    PF1
          cpi     A,13
          brne    L_152E
          or      A,A
          ret
;*   
; PC no branch 5327
L_1548:   brmi    PC+3
          jmp    RDERR
          mov     A,XL
          or      A,A
; PC no branch 5322
          breq    PC+3
          jmp    RDERR
          ldi     ZL,low(CEOF)
          ldi     ZH,high(CEOF)
          ldi     c_tmp,6
          st      Z,c_tmp
          ret
;*
;*
;*
;*     FILE CLOSE PRIMITIVE
;*
;*
FLCLZ:    ldi     ZL,low(CUFID)
          ldi     ZH,high(CUFID); ZERO THE USER'S FILE ID BYTE TO SHOW CLOSED
          ldi     c_tmp,0
          st      Z,c_tmp
          rcall   CLOFCB        ; CLOSE FOR GOOD
          lds     A,CSFID
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -5469
          call    cFCLOS
; PC no branch 5278
          brcc    PC+3
          jmp    CLERR
          ret
;*   
S_156B:   push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
; PC no jump 4348
          call    CCRLF
          rcall   TTON
          lds     A,CSFID
          ldi     c_tmp,'0'        ; ADI
          add     A,c_tmp
          sts     PtUMsg+18,A
          ldi     ZL,low(PtUMsg)
          ldi     ZH,high(PtUMsg)
; PC no jump 6283
          call    PRNT
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          cpi     A,4
; PC no branch 5238
          brcs    PC+3
          jmp    ACERR
          or      A,A
; PC no branch 5234
          brne    PC+3
          jmp    ACERR
          sts     CACRE,A
          dec     A
          ldi     ZL,low(TuRdMsg*2)
          ldi     ZH,high(TuRdMsg*2)
          breq    L_1597
          ldi     ZL,low(TuWrMsg*2)
          ldi     ZH,high(TuWrMsg*2)
; PC no jump 6257
L_1597:   call    pgm_PRNT
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
          ldi     XL,0
; PC no jump 6251
          call    PRN1
; PC no jump 4425
          call    IF2
; PC no jump 4296
          call    CRLF
          rcall   TTOFF
          ldi     ZL,low(0x0100)
          ldi     ZH,high(0x0100)
          sts     CBLKS,ZL
          sts     CBLKS+1,ZH
          lds     A,CSFID
          ldi     ZL,low(CFN)
          ldi     ZH,high(CFN)
; PC no jump -5555
          call    cFOPEN
; PC no branch 5189
          brcc    PC+3
          jmp    OPERR
          ldi     ZL,low(CEOF)
          ldi     ZH,high(CEOF)
          ldi     c_tmp,1
          st      Z,c_tmp
          ret
;*
;*
;*     THE  EXPRESSION  EVALUATOR
;*
;*


;NAME1:		BSM#MSCS
;PSHAS:		BSM#MSCS
;CONST:		BSM#VARC
;FLEN:		BSM#FUN1
;FCALL:		BSM#FUN1
;FASC:		BSM#FUN1
;FVAL:		BSM#FUN1
;TOPFP:		BSM#MSCS
;EATRP:		BSM#MSCS
;SCANC:		BSM#MSCS
;DIRT:		BSM#VARC
;TYERR:		BSM#ERR
;PSHCS:		BSM#MSCS
;FTXA:		BSM#MSCS
;SCONS:		BSM#VARC
;STOV:		BSM#MSCS
;EATL0:		BSM#MSCS
;POPFP:		BSM#MSCS
;FPOUT:		BSM#CVT2


;% start of file BSM#EXPR
;*
;*
;*
;*     EVALUATE AN EXPRESSION FROM TEXT
;*     HL TAKE OP TABLE ADDR OF PREVIOUS OPERATOR
;*     HL LEFT UNCHANGED
;*     RESULT VALUE LEFT ON TOP OF ARG STACK, ARGF LEFT TRUE
;*
;*
EXPRB:    ldi     ZL,low(OPBOL*2)
          ldi     ZH,high(OPBOL*2)
;*
EXPR:     push    ZL
          push    ZH            ; PUSH OPTBA
          ldi     ZL,low(-CMNDSP+SSIZE-10)
          ldi     ZH,high(-CMNDSP+SSIZE-10); CAUSE ERROR IF WITHIN 10 OF END O
          _DAD_SP
          ldi     XL,low(0x4953)
          ldi     XH,high(0x4953)
; PC no branch 5152
          brcs    PC+3
          jmp    ERROR         ; INTERNAL STACK TOO BIG
          eor     A,A
          sts     ARGF,A
          rjmp    EXP1A
;*   
EXPR1:    lds     A,ARGF
          or      A,A
          brne    EXPR2
;*   
; PC no jump 5547
EXP1A:    call    NAME1         ; LOOK FOR POSSIBLE VARIABLE NAME
          brcs    EXPR0         ; WASN'T VARIABLE
; PC no jump 5709
          call    PSHAS         ; PUT ON STACK
          rjmp    EXPR2
;*
; PC no jump 2621
EXPR0:    call    CONST
          brcc    EXPR2
          eor     A,A
          rcall   FUNC
          brcc    EXPR2
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z
          cpi     A,0xDB
; PC no branch 2001
          brne    PC+2
          rjmp    FLEN
          cpi     A,0xDC
; PC no branch 1966
          brne    PC+2
          rjmp    FCALL
          cpi     A,0x87
; PC no branch 2004
          brne    PC+2
          rjmp    FASC
          cpi     A,0x89
; PC no branch 2113
          brne    PC+3
          jmp     FVAL
          cpi     A,0xC8
          ldi     ZL,low(OPTAB*2)
          ldi     ZH,high(OPTAB*2)
; PC no branch 142
          brne    PC+2
          rjmp    XLPAR
;*
;*     ISN'T OR SHOULDN'T BE AN ARGUMENT
;*
EXPR2:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z
          cpi     A,0xC8        ; TOKCM
          brcs    EXPR8
          cpi     A,0xD8
          brcs    XOP
          cpi     A,0xED
; PC no branch 103
          brcc    PC+2
          rjmp    XBILT
;*
;*   ILLEGAL EXPRESSION CHARACTER
;*
EXPR8:    pop     ZH
          pop     ZL            ; GET OPTABA
          lds     A,ARGF
          or      A,A
          breq    PC+2          ; RNZ
          ret
; PC no jump 5050
          jmp     BSERR
;*
;*
XOP:      lds     A,ARGF        ; TEST FOR ARGF TRUE
          dec     A
          ld      A,Z           ; GET BACK TOKEN
          breq    XOP1
;*
;*   ARGF WAS FALSE, UNARY OPS ONLY POSSIBILITY
;*
          cpi     A,0xCC        ; MINRW
          breq    XOPM
;*
          cpi     A,0xD6        ; NOTRW
          breq    XOP1
;*
          cpi     A,0xCB        ; PLSRW
; PC no branch 5027
          breq    PC+3
          jmp    BSERR
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          rjmp    EXPR1
;*
XOPM:     ldi     A,0xC9        ; UMINUS
XOP1:     ldi     ZL,low(OPTAB*2)
          ldi     ZH,high(OPTAB*2)
          subi    A,0xC8    ;SUI; TOKCM
          rcall   OPADR
;          pop     YH
;          pop     YL            ; PREVIOUS OPTBA
	movw DPL,ZL

	pop ZH
	pop ZL
;          ld      A,Y
          lpm
		  mov A,r0
	;retore Y
	movw YL,ZL
	movw ZL,DPL
;         ld      c_tmp,Z
;         cp      A,c_tmp
    lpm
    cp      A,r0
          brcs    PC+2          ; RNC
          ret
;*
;*     INCREASING PRECEDENCE CASE
;*
          push    YL
          push    YH
          _XCHG
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          _XCHG                 ; GET IT BACK
          push    ZL
          push    ZH
          rcall   EXPR
          pop     ZH
          pop     ZL
;*
;*     HL HAS OPTBA ADDRESS
;*     SET UP ARGS AND PERFORM OPERATION ACTION
;*
XOP2:     push    ZL
          push    ZH
;          ld      A,Z
	lpm
	mov A,r0
; PC no jump 5559
          call    TOPFP
          movw     XL,YL
;*
;* AT THIS POINT: DE= # DE  FOR UNARY OPS
;*
          andi    A,1
          brne    XOP21
;*
;*     DECREMENT STACK POINTER BY ONE VALUE, BINARY CASE
;*
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          ldi     XL,low(FPSIZ)
          ldi     XH,high(FPSIZ); FPSIZ
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          movw     XL,ZL
;*
XOP21:    ldi     ZL,low(EXPR1)
          ldi     ZH,high(EXPR1)
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; CHANGE RETURN LINK
          adiw    ZL,1          ; SKIP OVER PRECEDENCE
 ;         ld      A,Z
 	lpm
	mov A,r0
          adiw    ZL,1
 ;         ld      ZH,Z
 	lpm
	mov ZH,r0
          mov     ZL,A
          push    ZL
          push    ZH
          _XCHG                 ; DE TO HL
          mov     YH,XH
          mov     YL,XL
;*
;*       AT THIS POINT: BC=DE # HL
;*
FLPAR:    ret                   ; BRANCH TO ACTION ROUTINE
;*
;*           BUILT IN FUNCTION PROCESSING
;*
XBILT:    adiw    ZL,1          ; EAT TOKEN
          sts     TXA,ZL
          sts     TXA+1,ZH
          subi    A,0xD8           ;SUI; TOKOP
          mov     XL,A
          lds     A,ARGF        ; BUILT IN FUNCTION MUST COME AFTER AN OPERATOR
          dec     A
; PC no branch 4942
          brne    PC+3
          jmp    BSERR
          ldi     ZL,low(OPBOL*2)
          ldi     ZH,high(OPBOL*2); DISPATCH TABLE FOR BUILT IN FUNCTIONS
          rcall   OPAD1         ; OPTBA TO HL
;*
XLPAR:    push    ZL
          push    ZH
; PC no jump 5634
          call    EATLP
          rcall   EXPRB
; PC no jump 5633
          call    EATRP
          pop     ZH
          pop     ZL            ; CODE FOR BUILT-IN FUNTION
          rjmp    XOP2
;*
;*          COMPUTE OPTABLE ADDRESS FOR OPERATOR IN ACC
;*
OPADR:    mov     XL,A
OPAD1:    ldi     XH,0
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; OPTAB ENTRY ADDR IS 3*     OP+BASE
          ret
;*
;*
;*
;*
;*     EVALUATE FUNCTION, AND LEAVE VALUE ON STACK
;*     SET CARRY IF NO FUNCTION CALL FOUND
;*
;*
FUNC:     ori     A,0x7F        ; ACCA SEPARATES NUMERIC FROM STRING
          mov     YH,A          ; SAVE FLAG
          ldi     A,0x84        ; FNRW
; PC no jump 5645
          call    SCANC
          brcc    PC+2          ; RC
          ret                   ; RETURN IF NO FN RW FOUND
; PC no jump 2481
          call    DIRT
          push    YL
          push    YH            ; STRING\NUMERIC FLAG
; PC no jump 5362
          call    FNAME
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; STRING NUMERIC FLAG
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          eor     A,XL
; PC no branch 4853
          brpl    PC+3
          jmp    TYERR
; PC no jump 2328
          call    STLK
; PC no branch 4835
          brcc    PC+3
          jmp    FDERR         ; USE OF UNDEFINED FUNCTION
          ld      YL,Z
          adiw    ZL,1
          ld      YH,Z
          push    YL
          push    YH
; PC no jump 5582
          call    EATLP
;*
;*  LOOP TO ASSIGN ARGS TO FORMALS
;*
          ldi     YL,low(-1)
          ldi     YH,high(-1)
; PC no jump 5491
          call    PSHCS         ; PUSH MARKER
          ldi     c_tmp,4
          st      Z,c_tmp       ; EFTYPE
; PC no jump 5216
          call    FTXA          ; TXA TO DEFINITION
;*
FUNC1:    rcall   VAR
; PC no jump 5210
          call    FTXA          ; TXA TO CALL
          breq    FUNC3         ; JUMP IF STRING CASE
;*
;*  NUMERIC CASE
;*
          push    ZL
          push    ZH            ; SYM TAB PTR
; PC no jump 5439
          call    PSHAS         ; SAVE OLD VALUE OF PARAMETER ON STACK
          ldi     YL,low(-3)
          ldi     YH,high(-3)
; PC no jump 5467
          call    PSHCS
          pop     YH
          pop     YL
          push    YL
          push    YH            ; SYMTAB PTR
          st      Z,YH          ; SAVE SYMTAB PTR ON CSTACK
          sbiw    ZL,1
          st      Z,YL
          sbiw    ZL,1
          ldi     c_tmp,3
          st      Z,c_tmp       ; ANTYPE
          rcall   EXPRB         ; GET NEW ARG VALUE
          pop     YH
          pop     YL            ; SYMTAB PTR
; PC no jump 5439
          call    POPA1         ; STORE ARG VALUE IN SYMTAB
;*
; PC no jump 5559
FUNC2:    call    GCI
          cpi     A,','
          brne    FUNC9         ; MUST BE END OF LIST
; PC no jump 5171
          call    FTXA          ; TXA TO DEF
; PC no jump 5557
          call    SCOMA         ;  SHOULD BE COMMA IN DEF NOW
; PC no branch 80
          brcc    PC+2
          rjmp    FUNC6
          rjmp    FUNC1
;*
;*
;*  STRING ARGUMENT CASE
;*
FUNC3:    ldi     A,0
          rcall   STASS
          rjmp    FUNC2
;*
;*
;*     FOUND NON COMMA IN CALL
;*
FUNC9:    cpi     A,')'
; PC no branch 4802
          breq    PC+3
          jmp    BSERR
; PC no jump 5146
          call    FTXA          ; TXA TO DEF
          ldi     A,')'
; PC no jump 5532
          call    SCANC
          brcs    FUNC6
          ldi     YL,low(-3)
          ldi     YH,high(-3)
; PC no jump 5402
          call    PSHCS
          pop     YH
          pop     YL            ; TEXT PTR TO CALL (AFTER ')')
          st      Z,YH
          sbiw    ZL,1
          st      Z,YL
          sbiw    ZL,1
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; TYPE OF FUNCTION CALL
          st      Z,A
;*
;*  NOW COPY STACK BLOCK TO TEMP STACK
;*
          ldi     ZL,low(-CMNDSP-3)
          ldi     ZH,high(-CMNDSP-3)
          _DAD_SP
          _XCHG
; PC no jump 5387
          call    PSHCS
          ldi     XL,low(CSPM1)
          ldi     XH,high(CSPM1)
          inc     YL
          inc     YL
          mov     YH,YL
;*
FUNC8:    ld      A,X
          st      Z,A
          sbiw    XL,1
          sbiw    ZL,1
          inc     YL
          brne    FUNC8
;*
          st      Z,YH          ; SAVE IFTERM
          sbiw    ZL,1
;*
          ldi     c_tmp,5
          st      Z,c_tmp       ; PUT TYPE ON STACK
          ldi     ARGL,low(CMNDSP)     ; LXI SP
          ldi     ARGH,high(CMNDSP)
          out     SPL,ARGL
          out     SPH,ARGH      ; SET UP INTERNAL STACK AGAIN
;*
;*  NOW DECIDE IF IT WAS A ONE-LINER OR MULTI-LINE FUNCTION
;*  NOTE: THE RETURN STATEMENT WILL RETURN TO OUR CALLER
;*
          ldi     A,0xD4        ; EQRW
; PC no jump 5482
          call    SCANC
; PC no branch -4697
          breq    PC+3
          jmp    IL1           ; GO EXECUTE FIRST STATEMENT OF FUNCTION
; PC no jump -2784
          jmp     RETRN         ; GO COMPUTE VALUE TO RETURN
;*
;*
;*  COME HERE WHEN A  EXPECTED ')' WAS NOT FOUND IN DEF
;*
; PC no jump 5082
FUNC6:    call    FTXA          ; TXA TO CALL
          ldi     XL,low(0x414D)
          ldi     XH,high(0x414D)
; PC no jump 4732
          jmp     ERROR         ; ARLIST MISMATCH
;*
;*
;*     STRING EXPRESSION EVALUATION
;*     ACC IS STRING CONST TERMINATION CHAR, 0=SAME AS DOUBLE QUO
;*     IF ACC=0 THEN GENERAL STRING EXPR'S FOUND, ELSE ONLY SINGL
;*     LEAVES STRING ON ARG STACK
;*     RETURN LENGTH IN HL
;*
SEXPG:    eor     A,A           ; USE THIS ENTRY POINT FOR GENERAL EXPRESSIONS
;*
SEXPR:    ldi     XL,low(0x0000)
          ldi     XH,high(0x0000); INITIAL SIZE OF RESULT STRING
          push    XL
          push    XH            ; ACCUMULATING LENGTH OF RESULT
          or      A,A           ; SET Z CONDITION FROM ACCUMULATOR
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; STRING TERMINATION CHAR
SEXP0:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; GET ARG FOR SCONS
; PC no jump 2240
          call    SCONS
          brcc    SEXP1         ; JMP IF STRING CONSTANT FOUND
;*   
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
; PC no branch 4709
          breq    PC+3
          jmp    BSERR         ; EXPRESSION MUST BE SINGLE STRING CONST
;*   
; PC no jump 5428
          call    GC
          cpi     A,0x86        ; CHRRW
; PC no branch 77
          brne    PC+2
          rjmp    ACHR          ; CHR FUNCTION
;*   
          cpi     A,0x8A        ; STRRW
; PC no branch 96
          brne    PC+2
          rjmp    ASTR
;*   
          cpi     A,0x88        ; ERRRW
; PC no branch 136
          brne    PC+2
          rjmp    AERR
;*   
          ldi     A,0xFF        ; SFTYPE
          rcall   FUNC
          brcc    SEXP3
;*   
          rcall   VAR
; PC no branch 4637
          breq    PC+3
          jmp    TYERR
;*
;*  CONCATENATE STRING TO STACK
;*
SEXP1:    mov     A,YH
          or      A,YL
          breq    SEXP3         ; STRING WAS ZERO LENGTH
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          mov     XH,ZH
          mov     XL,ZL         ; SAVE BEGINNING OF NEW STRING AREA IN BC
; PC no jump 2253
          _DSUB
; PC no jump 5327
          call    STOV          ; WILL STRING FIT
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; ALLOCATE NEW STRING IN TEMP AREA
          push    YL
          push    YH            ; SIZE
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
;*
;*  COPY STRING TO TEMP STACK
;*
SEXP2:    ld      A,Z
          st      X,A
          sbiw    XL,1
          adiw    ZL,1
          sbiw    YL,1
          mov     A,YH
          or      A,YL
          brne    SEXP2
          pop     YH
          pop     YL            ; SEXP3 EXPEXTS SIZE IN D
;*
;*   STRING COPIED TO STACK, NOW SEE IF MORE TO DO
;*
SEXP3:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; TEST THAT  SINGLE STRING CONST
          pop     ZH
          pop     ZL            ; ACCUMULATING SIZE
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          push    ZL
          push    ZH
          brne    SEXP4         ; IF ONLY STR CONST ALLOWED
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          ldi     A,0xCB        ; PLSRW
; PC no jump 5368
          call    SCANC
; PC no branch -95
          brcs    PC+2
          rjmp    SEXP0
          pop     ZH
          pop     ZL            ; GET RID OF SCONS ARG
SEXP4:    pop     ZH
          pop     ZL            ; RETURN LENGTH IN HL
          ret
;*
;*     HANDLES THE CHR$ FUNCTION FOR SEXPR
;*
; PC no jump 5320
ACHR:     call    EATL0         ; EAT THE CHR$ RW AND THE LEFT PAREN
; PC no jump 2250
          call    PFIXE
          push    YL
          push    YH
          ldi     YL,low(-1)
          ldi     YH,high(-1)
; PC no jump 5228
          call    PSHCS
          pop     YH
          pop     YL
          st      Z,YL          ; PUT THE SINGLE CHARACTER ON STACK
; PC no jump 5313
          call    EATRP
          ldi     YL,low(1)
          ldi     YH,high(1); SET UP FOR SEXPR3
          rjmp    SEXP3
;*
;*     HANDLES THE STR$ FUNCTION
;*
; PC no jump 5296
ASTR:     call    EATL0
          rcall   EXPRB
; PC no jump 5298
          call    EATRP
;*
; PC no jump 5188
          call    POPFP         ; POP THE VALUE
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; ADDR OF WHERE ARG WAS ON STACK
; PC no jump 2445
          call    FPOUT
;*
          lds     A,CFORM
          cpi     A,'#'         ; FREE FORMAT?
          brne    ASTR1         ; NO
;*   
ASTR0:    ld      A,Z
          adiw    ZL,1
          dec     XH
          cpi     A,' '
          breq    ASTR0         ; EAT SPACES
          sbiw    ZL,1
          inc     XH
;*
;*  SIZE IS IN B, ADDRESS OF FIRST CHAR IN HL
;*
ASTR1:    mov     YL,XH
          ldi     YH,0
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          rjmp    SEXP1
;*
;*  ERR FUNCTION
;*
; PC no jump 5251
AERR:     call    EATL0
; PC no jump 2181
          call    PFIXE
          mov     A,YH
          or      A,YL
; PC no branch 4498
          breq    PC+3
          jmp    OBERR
; PC no jump 5248
          call    EATRP
;*   
          ldi     YL,low(22)
          ldi     YH,high(22); LENGTH OF ERRBUF
          ldi     ZL,low(SHORT)
          ldi     ZH,high(SHORT)
          ldi     A,' '
          ld      c_tmp,Z
          cp      A,c_tmp
          breq    AERR0
          ldi     YL,low(0x0008)
          ldi     YH,high(0x0008)
AERR0:    ldi     ZL,low(LSTERR)
          ldi     ZH,high(LSTERR); ERRBUF  ADDR OF SAME
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          rjmp    SEXP1
;*
;*
;*     STRING ASSIGNMENT SUBROUTINE
;*     ACC=0 IF GENERAL EXPRESSION OK, ELSE ACCUMULATOR CONTAINS
;*      TERMINATION CHARACTER FOR A CONSTANT STRING
;*     THE REGISTERS AND PSW ARE SET UP AS BY A VAR RETURN
;*
;*
STASS:    push    ZL
          push    ZH            ; BASE ADDRESS--"LET" ENTERS AT STAS1 W/ HL ON STACK
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; TERMINATION CHARACTER AND SUBSTRING FLAG
          push    XL
          push    XH            ; SYMTAB PTR TO LG
          lds     ZL,STRMX     ;LHLD
          lds     ZH,STRMX+1
          push    ZL
          push    ZH
          rcall   SEXPR         ; ACC STILL HAS TERMINATION FLAG
          _XCHG                 ; SOURCE LENGTH TO DE
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          ldi     ZL,low(0x0001)
          ldi     ZH,high(0x0001)
          _DAD_SP               ; PUTS ADDRESS OF STRMX(ON STACK) IN HL
; PC no jump 4683
          call    DCMP1
          pop     ZH
          pop     ZL            ; REMOVE STRMX FROM STACK
          brcs    STAS2
          _XCHG
;*
;* THE SMALLER OF SOURCE LENGTH AND DESTINATION MAX IS IN DE
;*
STAS2:    pop     ZH
          pop     ZL            ; SYMTAB PTR TO LOG SIZE IN DESTINATION
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET C CONDITION FOR SUBSTRING TEST
          brcs    PC+3          ; CNC
          _DSTOR         ; MODIFY LOGICAL LENGTH ONLY IF NOT SUBSTRING
          pop     XH
          pop     XL            ; BASE ADDRESS OF RESULT
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
;*
;* HL HAS FIRST CHAR ON ARG STACK (IF ANY)
;* DE HAS COUNT (POSSIBLY 0)
;* BC HAS BASE ADDRESS OF DEST
;*
          mov     A,YH
STAS4:    or      A,YL          ; FASTER LOOP IF WE TEST FIRST
          brne    PC+2          ; RZ
          ret
STA4A:    ld      A,Z
          st      X,A
          sbiw    ZL,1
          adiw    XL,1
          sbiw    YL,1
          mov     A,YH
          or      A,YL
          brne    STA4A
          ret
;% end of file BSM#EXPR
;*
;*
;*     THE  LOGICAL  OPERATORS
;*
;*
;ZEX:		BSM#MATH
;VCPY1:		BSM#MSCS
;% start of file BSM#LGIC
;*
;*
;*
;*   ACTION ROUTINES FOR RELATIONAL OPERATORS
;*
;*
AGT:      rcall   RELOP
          brcs    RTRUE
;*
RFALSE:   mov     ZH,XH
          mov     ZL,XL
          rjmp    ZEX           ; STORE A ZERO
;*
ALT:      rcall   RELOP
          breq    RFALSE
          brcs    RFALSE
;*
RTRUE:    mov     YH,XH         ; BC TO DE
          mov     YL,XL
; PC no jump 4998
          jmp     VCPY1         ; FPONE FOR TRUE
;*
AEQ:      rcall   RELOP
          breq    RTRUE
          rjmp    RFALSE
;*
ANE:      rcall   RELOP
          brne    RTRUE
          rjmp    RFALSE
;*
AGE:      rcall   RELOP
          breq    RTRUE
          brcs    RTRUE
          rjmp    RFALSE
;*
ALE:      rcall   RELOP
          breq    RTRUE
          brcc    RTRUE
          rjmp    RFALSE
;*
;*
;*
;*         COMMON ROUTINE FOR RELATIONAL OPERATOR ACTION
;*
;*  RESULT ADDRESS IN BC, SAVED
;*  LEFT ARG ADDRESS IN DE, SAVED
;*  RIGHT ARG ADDRESS IN HL
;*  RETURN:  SIGN=1 ==>  GT
;*           ZERO=1 ==>  EQ
;*
RELOP:    
          in r25,SREG
		  sbiw    ZL,1
          sbiw    YL,1          ; POINT TO SIGN BYTE
          out SREG,r25      ;% probably not needed
		  ld      A,Y           ; GET SIGN
          ld      c_tmp,Z
          cp      A,c_tmp       ; COMPARE THE TWO
          breq    PC+2          ; RNZ
          ret                   ; NOT EQUAL...THE FLAGS ARE SET
          in r25,SREG
          adiw    ZL,1
          adiw    YL,1          ; POINT BACK TO THE EXP
          out SREG,r25		;% these flags probably need protecting
;*
          or      A,A
          brne    RLOO1         ; BOTH NOT ZERO
          _XCHG                 ; EXCHANGE ORDER OF ARGS
;*
RLOO1:    ld      A,Y
          ld      c_tmp,Z
          sub     A,c_tmp
          breq    PC+2          ; RNZ
          ret
;*
          push    XL
          push    XH
          ldi     XL,low(0xFFFB)
          ldi     XH,high(0xFFFB); -FPSIZ+1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          _XCHG                 ; HL POINT RIGHT...SAVE IN DE
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; FIX DE
          _XCHG                 ; AND ADJUST
          inc     XL
;*
RCOMP:    ld      A,Y+
          ld      c_tmp,Z+
          sub     A,c_tmp
          brne    RLOP1
;          adiw    ZL,1
;          adiw    YL,1
          inc     XL
          brne    RCOMP
;*
RLOP1:    pop     XH
          pop     XL
          ret
;*
;*
;*
;*
;*  BOOLEAN OPERATORS
;*    AND OR NOT
;*          HL AND DE POINT TO ARGS
;*          WE ASSUME DE=BC AND THE POINT TO TOS+1 (ONE DOWN)
;*                 BC = DE OP HL
;*
AAND:     ld      A,Y
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; ALREADY ZERO
          ld      A,Z
          or      A,A
; PC no branch -106
          brne    PC+2
          rjmp    RFALSE
;*
;*    COPY FPONE TO DESTINATION
;*
AAND1:    mov     YH,XH
          mov     YL,XL
; PC no jump 4906
          jmp     VCPY1
;*
AOR:      ld      A,Z
          or      A,A
          brne    AAND1         ; MAKE 1
          ld      A,Y
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; ALREADY ZERO
          rjmp    AAND1         ; MAKE 1
;*
ANOT:     ld      A,Y
          or      A,A
          breq    AAND1         ; WAS 0 SET TO 1
          rjmp    RFALSE        ; WAS NON 0 SET TO 0
;% end of file BSM#LGIC
;*
;*
;*     THE  MATH  PACKAGE
;*
;*
; code macro for decimal accumulator adjust 
DAA:
; the 8 bit number i n the accumulator is adjusted to form two
; four bit Binary Coded Decimal digits by the following process:
;
; 1. If the value of the lease significant 4 bits
; of the accumulator is greater than 9 or if the
; AC flag is set, 6 is added to the accumulator
; 2. If the value of the most significant 4 buts
; of the accumulator is greater than 9 or if the CY
; flag is set 6 is added to the most significant
; four bits of the accumulator
; Note: All flags are affected

;	push A		; r16
;	push ARGL	; r17
;	push ARGH	; r18
;	push r25    ; r19
	
;	push A 
	mov ARGL,A
	mov ARGH,A
	in r25,SREG
	andi r25, (1<<SREG_C)
	
	clc
	brhs DAA_adjlo		; Half carry set
	andi ARGL,0x0F
	cpi ARGL,10
	brlo DAA_hi
DAA_adjlo:
	ldi ARGL, 6
	add A,ARGL
	
DAA_hi:
	tst r25
	brne DAA_adjhi
	mov ARGL,ARGH
	cpi ARGL,0x9A
	brlo DAA_nadjhi
DAA_adjhi:
	ldi ARGL,0x60
	add A,ARGL
	sec
	rjmp DAA_end
DAA_nadjhi:
	clc
	
DAA_end:
;	pop r25
;	pop ARGH
;	pop ARGL
;	pop A
	
ret
;% start of file BSM#MATH
;*
;*
;*    THIS PROGRAM IS A FOUR FUNCTION FLOATING POINT BCD
;*    MATH PACKAGE.
;*
;* EACH FUNCTION MAY BE EXPRESSED AS: BC=DE # HL
;*
;*     <BC> = ADDRESS OF RESULT
;*     <DE> = ADDRESS OF 1ST ARGUMENT
;*     <HL> = ADDRESS OF 2ND ARGUMENT
;*       #    IS ONE OF THE FUNCTIONS: +,-,X,/,^
;*
;* ALL ADDRESSES ON ENTRY, POINT TO THE EXPONENT PART OF
;* THE FLOATING POINT NUMBER.
;*
;* EACH FLOATING POINT NUMBER CONSISTS OF (2*DIGIT) PACKED
;* DECIMAL DIGITS, A SIGN AND A BIASED BINARY EXPONENT.  THE
;* EXPONENT RANGE IS 10**-127 TO 10**127.
;*
;* THE NUMBER ZERO IS REPRESENTED BY THE EXPONENT 0.
;* OF DECIMAL DIGITS STARTING AT THE LOW ORDER ADDRESS.
;*
;* ALL NUMBERS ARE ASSUMED TO BE NORMALIZED.  THAT IS EACH
;* NUMBER CAN BE REPRESENTED AS  F**E.
;*    WHERE  .1<=F<1.0 AND E IS THE EXPONENT
;*
;*
;*
;*
;*
;*        FLOATING POINT ADDITION
;*
FADD:     push    XL
          push    XH            ; SAVE RESULT ADDRESS
          rcall   EXPCK         ; FETCH AND ALIGN ARGUMENTS.
;*  THE NUMBER WITH THE LARGER EXPONENT IS POINTED TO BY DE.
;*  THE # WITH THE SMALLER EXP IS IN BUF AND SIGN.
          ldi     XL,0          ; DON'T CARE IF ARGUMENTS ARE REVERSED.
;*
ADSUM:    sbiw    YL,1          ; MOVE TO SIGN OF LARGER NUMBER.
          _XCHG
          lds     A,SIGN        ; SIGN OF SMALLER NUMBER.
          ld      c_tmp,Z
          eor     A,c_tmp       ; DETERMINE WHETHER WE ADD OR SUBTRACT. 0-ADD;1-SUB.
          mov     XH,A
          ld      A,Z           ; SIGN OF RESULT IS SIGN OF LARGER # EXCEPT WHEN
          _XCHG                 ; SUBTRACTING AND THE ARGUMENTS WERE REVERSED, I.E.
          sbiw    YL,1          ; SMALLER MINUS LARGER.  THEN SIGN OF RESULT IS THE
          eor     A,XL          ; OPPOSITE OF THE SIGN OF THE LARGER NUMBER.
          sts     SIGN,A        ; REMEMBER THAT.
;% this codeblock is quite different in the
;% Revised version on the PT DOS source disks
;% where it is located below EXPC3
;*
;*
;*  DETERMINE IF WE SHOULD ROUND FOR FADD OR FSUB
;*  SET THE CARRY IF THE ROUNDING DIGIT IS >= 5.
;*
SETROUND: ldi     ZL,low(RCTRL)
          ldi     ZH,high(RCTRL); DETERMINE WHICH NIBBLE THE DIGIT IS IN.
          ld      A,Z           ; % not in revision
          or      A,A           ; IF 0 THEN HIGH (LEFT) NIBBLE.
		  clc	  ; clear CY for safety
		  in r25,SREG
          adiw    ZL,1          ; % not in revision
          out SREG,r25
		  ld      A,Z           ; % not in revision
;% LDA RDIGI  THE ROUNDING DIGIT (AND SOME OTHER NIBBLE).
          breq    SETR1         ; THE DIGIT IS IN THE RIGHT PLACE.
;*
          swap A
;          lsl     A      ;RLC -- check cary rotate; ROTATE THE CORRECT DIGIT TO THE HIGH NIBBLE.
;          lsl     A      ;RLC -- check cary rotate; WE DON'T CARE WHAT HAPPENS TO THE LOW NIBBLE.
;          lsl     A      ;RLC -- check cary rotate; ONE MORE
;          lsl     A      ;RLC -- check cary rotate; 4 SHIFTS SWAPS NIBBLES.
;*
SETR1:    ldi     c_tmp,0xB0        ; ADI
          add     A,c_tmp       ; THIS SETS CARRY IF HIGH NIBBLE IS >=5.
;* THAT'S WHAT WE WANTED.
;*
          mov     A,XH          ; GET THE OPERATION INDICATOR.
          ror     A             ; % revised code says: ORA A  TEST IT. 
; PC no branch 79
          brcc    PC+2
          rjmp    ADS1          ; GO SUBTRACT. % revised to JNZ
          rol     A             ; % not in revision
;*
;*  THE ADD
;*
          rcall   S_ADD         ; CARRY IS CLEAR ON ENTRY; INDICATES CARRY OUT ON EXIT.
          brcc    ADS2          ; RESULT FIT IN DIGIT*2 BYTES.
;*
          ldi     XL,low(0x0405)
          ldi     XH,high(0x0405); 4*256+DIGIT+1  B <= 4 (NUMBER OF BITS TO SHIFT RIGHT)
;*           ^^--the above is off by 1
          rcall   RIGH1         ; THIS ENTRY TO RIGHT ASSUMES C IS SET. (QUICKER)
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)  ; ADJUST EXPONENT UP SINCE WE DECREASE THE # BY SHIFT
;*
          _INR_M                ; ROUNDING OVERFLOW SO PUT 1 IN BYTE ABOVE BUF.
; PC no branch 967
          brne    PC+2
          rjmp    OVER          ; GO ADJUST AGAIN.
;*
ADS2:     pop     XH
          pop     XL
;*
STORE:    ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
STORO:    ldi     YL,6          ; DIGIT+2
STOR1:    ld      A,Z
          st      X,A
		  in r25,SREG
          sbiw    ZL,1
          sbiw    XL,1
          out SREG,r25          ; probably do not need to protect here
          dec     YL
          brne    STOR1
          ret                   ; EXIT FROM ROUTINES
;*
;*
;*     CLEAN UP STACK--ZERO SIGN AND EXPONENT
;*
;*
;*
ZERE0:    pop     ZH
          pop     ZL            ; RESULT ADDRESS
ZEX:      ldi     XL,6          ; DIGIT+2  NUMBER TO CLEAR
;*
CLEAR:    eor     A,A
CLER1:    st      Z,A
          sbiw    ZL,1
          dec     XL
          brne    CLER1
          ret
;*
;*
;*
;*   ADD ROUTINE
;*
S_ADD:    ldi     ZL,low(BUF+DIGIT-1)
          ldi     ZH,high(BUF+DIGIT-1)
          ldi     XH,4
ADD1:     ld      A,Y
          ld      c_tmp,Z
          adc     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
		  in      r25,SREG	;% protect cary
          sbiw    ZL,1
          sbiw    YL,1
		  out SREG,r25		;%unprotect cary
          dec     XH
          brne    ADD1
          brcs    PC+2          ; RNC
          ret
          _INR_M
          ret
;*
;*
;*          FLOATING POINT SUBTRACTION
;*
FSUB:     push    XL
          push    XH
          rcall   EXPCK         ; GET ARGUMENTS
          lds     A,SIGN
          ldi     c_tmp, 1
          eor     A,c_tmp       ; COMPLEMENT SIGN
          sts     SIGN,A
          rjmp    ADSUM
;*
ADS1:     rol     A             ; CALL SETROUND  SET THE CARRY IF THE ROUNDING DIGIT >=5.
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25      ; IN SUBTRACTION THE BORROW (CARRY) HAS THE REVERSE SENSE.
;* SUB
          ldi     ZL,low(BUF+DIGIT-1)
          ldi     ZH,high(BUF+DIGIT-1)
          ldi     XL,low(DIGIT*256+0x99)
          ldi     XH,high(DIGIT*256+0x99); DIGIT*256+99H
;*
SUB1:     mov     A,XL
          ldi     c_tmp,0     ; ACI
          adc     A,c_tmp
          ld      c_tmp,Z
          sub     A,c_tmp
          _XCHG
          ld      c_tmp,Z
          add     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          _XCHG
          st      Z,A
		  in r25,SREG
          sbiw    ZL,1
          sbiw    YL,1
          out SREG,r25		; protect cary flag
          dec     XH
          brne    SUB1
          brcs    ADS4
          ldi     ZL,low(SIGN)
          ldi     ZH,high(SIGN)
          ld      A,Z           ; GET SIGN
          ldi     c_tmp, 1
          eor     A,c_tmp       ; COMPLEMENT
          st      Z,A
;*
          sbiw    ZL,1   ;% subtract in loop will change flags
          ldi     XL,low(DIGIT*256+0x9A)
          ldi     XH,high(DIGIT*256+0x9A); 0x49A
;*
ADS3:     mov     A,XL          ; GET 9AH
          ld      c_tmp,Z
          sbc     A,c_tmp       ; COMPLEMENT RESULT
          ldi     c_tmp,0        ; ADI
          add     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
		  in r25,SREG
          sbiw    ZL,1
          out SREG,r25
          dec     XH
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brne    ADS3
;*
ADS4:     ldi     ZL,low(BUF)
          ldi     ZH,high(BUF)
          ldi     XL,low(0x0004)
          ldi     XH,high(0x0004); DIGIT
          rjmp    ADS5
;*
;*
ADS5A:    
          in r25,SREG
		  adiw    ZL,1
          out SREG,r25
		  inc     XH
          inc     XH
          dec     XL
; PC no branch -109
          brne    PC+2
          rjmp    ZERE0
;*
ADS5:     ld      A,Z
          or      A,A
          breq    ADS5A
          cpi     A,16
          brcc    ADS9
          inc     XH
;*
ADS9:     ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          ld      A,Z
          sub     A,XH
; PC no branch -128
          brne    PC+2
          rjmp    ZERE0
; PC no branch -131
          brcc    PC+2
          rjmp    ZERE0
          st      Z,A
          mov     A,XH
          lsl     A      ;RLC -- check cary rotate
          lsl     A      ;RLC -- check cary rotate
          mov     XH,A
          rcall   LEFT
          rjmp    ADS2
;*
;*
;*
;*            FLOATING POINT MULTIPLY
;*
S_FMUL:   push    XL
          push    XH
FMULH:    ld      A,Z
          or      A,A           ; ARGUMENT = 0?
; PC no branch -148
          brne    PC+2
          rjmp    ZERE0
          ld      A,Y
          or      A,A           ; ARGUMENT = 0?
; PC no branch -153
          brne    PC+2
          rjmp    ZERE0
          ld      c_tmp,Z
          add     A,c_tmp       ; FORM RESULT EXPONENT
; PC no branch 792
          brcc    PC+2
          rjmp    FMOVR
; PC no branch -160
          brmi    PC+2
          rjmp    ZERE0
;* JMP FMUL1   %- revised code
;* FMOVR JM OVER % moved to just before OVER
;*
FMUL1:    subi    A,0x80           ;SUI; 128-1  REMOVE EXCESS BIAS
          sts     EXP,A         ; SAVE EXPONENT
          sbiw    YL,1
          sbiw    ZL,1
          ld      A,Y
          ld      c_tmp,Z
          eor     A,c_tmp       ; FORM RESULT SIGN
          sbiw    ZL,1
          sbiw    YL,1
          push    ZL
          push    ZH
          ldi     ZL,low(SIGN)
          ldi     ZH,high(SIGN) ; GET SIGN ADDRESS
          st      Z,A           ; SAVE SIGN
          sbiw    ZL,1
          eor     A,A
          ldi     XL,low(((DIGIT+2)*256)+DIGIT)
          ldi     XH,high(((DIGIT+2)*256)+DIGIT)
;*
FMUL2:    st      Z,A           ; ZERO WORKING BUFFER
          sbiw    ZL,1
          dec     XH
          brne    FMUL2
          ldi     ZL,low(HOLD1+DIGIT)
          ldi     ZH,high(HOLD1+DIGIT)
;*
;* GET MULTIPLIER INTO HOLDING REGISTER
;*
FMUL3:    ld      A,Y
          st      Z,A           ; PUT IN REGISTER
          sbiw    ZL,1
          sbiw    YL,1
          dec     XL
          brne    FMUL3
;*
          st      Z,XL
          sbiw    ZL,1
          ldi     XH,0xFA       ; 250  SET LOOP COUNT
;*   
FMUL4:    ldi     YL,low(0x0005)
          ldi     YH,high(0x0005); DIGIT+1
          mov     XL,YL
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; H,L=NEXT HOLDING REGISTER
          inc     XH
; PC no branch 94
          brmi    PC+2
          rjmp    FMUL8         ; FINISHED
;*
FMUL5:    ld      A,Y           ; GET DIGITS
          adc     A,A           ; TIMES 2
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A           ; PUT IN HOLDING REGISTER
          in r25,SREG		;% protect cary
		  sbiw    YL,1
          sbiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    FMUL5
;*
          inc     XH            ; INCREMENT LOOP COUNT
          brne    FMUL4
;*
;* FORM 10X BY ADDING 8X AND 2X
;* FIRST GET 8X
;*
          in r25,SREG		;% protect cary
          adiw    ZL,1
		  out SREG,r25
          ldi     YL,low(HOLD5)
          ldi     YH,high(HOLD5); NEXT HOLDING REGISTER
          ldi     XL,low(((DIGIT+1)*256)+DIGIT+1)
          ldi     XH,high(((DIGIT+1)*256)+DIGIT+1)
;*
FMUL6:    ld      A,Z+
          st      Y+,A
;          adiw    ZL,1
 ;         adiw    YL,1
          dec     XL
          brne    FMUL6
;*
          ldi     ZL,low(HOLD2+DIGIT)
          ldi     ZH,high(HOLD2+DIGIT);   GET 2X
          in r25,SREG		;% protect cary
          sbiw    YL,1
		  out SREG,r25
;*
FMUL7:    ld      A,Y
          ld      c_tmp,Z
          adc     A,c_tmp       ; FORM 10X
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Y,A
          in r25,SREG		;% protect cary
          sbiw    YL,1
          sbiw    ZL,1
		  out SREG,r25
          dec     XH
          brne    FMUL7
;*
          ldi     XH,0xF9       ; 249
          _XCHG
          rjmp    FMUL4
; CRM copyright text was here
FMUL8:    _XCHG
          in r25,SREG		;% protect cary
          adiw    ZL,1
		  out SREG,r25
          ldi     c_tmp,5
          st      Z,c_tmp       ; SET NEXT LOOP COUNT
          rjmp    FMUL9
;*
;*  ROTATE RIGHT 1 BYTE
;*
FMU12:    ldi     XL,low(8*256+DIGIT+1)
          ldi     XH,high(8*256+DIGIT+1); 0x0805
          rcall   RIGH1
;*
;*   PERFORM ACCUMULATION OF PRODUCT
;*
FMUL9:    pop     XH
          pop     XL            ; GET MULTIPLIER
          ldi     ZL,low(HOLD8+DIGIT+1)
          ldi     ZH,high(HOLD8+DIGIT+1); HOLD8+DIGIT+1
          _DCR_M                ; DECREMENT LOOP COUNT
          breq    FMU14         ; FINISHED
          ld      A,X
          in r25,SREG		;% protect cary
          sbiw    XL,1
          push    XL
          push    XH
          sbiw    ZL,1
		  out SREG,r25
          _XCHG
          rjmp    FMU10
;*
;*
FMU11:    mov     XL,A
          clc                   ;or      A,A     ; CLEAR CARRY
          rcall   S_ADD         ; ACCUMULATE PRODUCT
          ld      A,Y
          ld      c_tmp,Z
          add     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
          mov     A,XL
          in r25,SREG		;% protect cary
          sbiw    YL,1
		  out SREG,r25
;*
;*
FMU10:    add     A,A           ; CHECK FOR BIT IN CARRY
          brcs    FMU11         ; FOUND A BIT
          breq    FMU12         ; ZERO - FINISHED THIS DIGIT
          ldi     ZL,low(0xFFFB)
          ldi     ZH,high(0xFFFB); -DIGIT-1
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; POINT TO NEXT HOLDING REGISTER
          _XCHG
          rjmp    FMU10
;*
;*
;*
FMU14:    lds     A,BUF
          andi    A,0xF0        ; CHECK IF NORMALIZED
          breq    FMU17
          mov     A,YH
          andi    A,0xF0
          ldi     ZL,low(BUF+DIGIT-1)
          ldi     ZH,high(BUF+DIGIT-1)
          rjmp    FMU18
;*
FMU17:    ldi     XL,low(0x0405)
          ldi     XH,high(0x0405); 4*256+DIGIT+1
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          _DCR_M
; PC no branch -389
          brne    PC+2
          rjmp    ZERE0
          rcall   LEFTA         ; NORMALIZE
          mov     A,YH          ; GET DIGIT SHIFTED OFF
;*
;* PERFORM ROUNDING
;*
          swap A
;          lsr     A      ;RRC -- check cary rotate
;          lsr     A      ;RRC -- check cary rotate
;          lsr     A      ;RRC -- check cary rotate
;          lsr     A      ;RRC -- check cary rotate
FMU18:    cpi     A,'P'         ; 0x50
          brcs    FMU16
          inc     A
          andi    A,15
          ldi     XL,4          ; DIGIT
;*
FMU15:    ld      c_tmp,Z
          adc     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
          ldi     A,0
          in r25,SREG		;% protect cary
          sbiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    FMU15
;*
;* CHECK FOR ROUNDING OVERFLOW
;*
; PC no branch -435
          brcs    PC+2
          rjmp    ADS2          ; NO OVERFLOW
          in r25,SREG		;% protect cary
          adiw    ZL,1
		  out SREG,r25
          ldi     c_tmp,16
          st      Z,c_tmp
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          _INR_M
; PC no branch -445
          breq    PC+2
          rjmp    ADS2
          rjmp    OVER
;*
;* ROUNDING NOT NEEDED
;*
FMU16:    andi    A,15
          ld      c_tmp,Z
          add     A,c_tmp
          st      Z,A
          rjmp    ADS2
;*
;*
;*           FLOATING POINT DIVISION
;*
FDIV:     push    XL
          push    XH
          ld      A,Z           ; FETCH DIVISOR EXP
          or      A,A           ; DIVIDE BY 0?
; PC no branch 3800
          brne    PC+3
          jmp    DZERR
;* MOV B,A  SAVE THE DIVISOR EXP.
          ld      A,Y
          or      A,A           ; DIVIDEND = 0?
; PC no branch -451
          brne    PC+2
          rjmp    ZERE0
;* DCR B
;* SUB B
          ld      c_tmp,Z
          sub     A,c_tmp       ; % replaced by code comment out above
          brcs    DIVUN
; PC no branch 494
          brpl    PC+2
          rjmp    OVER
          rjmp    FDI1
; PC no branch -464
DIVUN:    brmi    PC+2
          rjmp    ZERE0
;*
FDI1:     ldi     c_tmp,0x81        ; ADI
          add     A,c_tmp       ; FORM QUOTIENT EXP
          sts     EXPD,A       ;  alternative equ is HOLD2
          _XCHG
          push    YL
          push    YH
          rcall   LOAD          ; FETCH DIVIDEND
          pop     YH
          pop     YL
          _XCHG
          lds     A,SIGN
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
          ld      c_tmp,Z
          eor     A,c_tmp       ; FORM QUOTIENT SIGN
          sts     SIGND,A ; HOLD1+DIGIT or XSIGND? REMEMBER IT IN TEMPORARY SIGN LOCATION.
          _XCHG
		  in r25,SREG
          sbiw    YL,1
		  out SREG,r25
          ldi     XL,low(HOLD1)
          ldi     XH,high(HOLD1)
DIV0:     ldi     ZL,8          ; DIGIT+DIGIT+1  1 EXTRA FOR ROUNDING DIGIT.
;*   
DIV1:     push    XL
          push    XH
          push    ZL
          push    ZH
          ldi     XL,0          ; QUOTIENT DIGIT = 0
;*
DIV3:     sec                   ; SET CARRY
          ldi     ZL,low(BUF+DIGIT-1)
          ldi     ZH,high(BUF+DIGIT-1)
          ldi     XH,4          ; DIGIT
;*
DIV4:     ldi     A,0x99
          ldi     c_tmp,0     ; ACI
          adc     A,c_tmp
          _XCHG
          ld      c_tmp,Z
          sub     A,c_tmp
          _XCHG
          ld      c_tmp,Z
          add     A,c_tmp
          rcall   DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
 		  in r25,SREG
          sbiw    ZL,1
          sbiw    YL,1
          out SREG,r25
          dec     XH
          brne    DIV4
;*
          ld      A,Z
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          sbci    A,0
          st      Z,A
          ror     A
          ldi     ZL,low(0x0004)
          ldi     ZH,high(0x0004); DIGIT
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
          inc     XL            ; INCREMENT QUOTIENT
          rol     A
          brcc    DIV3
;*
          clc     ;or      A,A           ; CLEAR CARRY
          rcall   S_ADD         ; RESTORE DIVIDEND
          ldi     ZL,low(0x0004)
          ldi     ZH,high(0x0004); DIGIT
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
          push    XL
          push    XH
          ldi     XL,low(0x0405)
          ldi     XH,high(0x0405); 4*256+DIGIT+1
          rcall   LEFTA         ; SHIFT DIVIDEND
          pop     XH
          pop     XL
          dec     XL
          pop     ZH
          pop     ZL
          mov     ZH,XL
          pop     XH
          pop     XL
          mov     A,ZL
          brne    DIV5
;*
          cpi     A,8           ; DIGIT+DIGIT+1
          brne    DIV5
;*
          ldi     ZL,low(EXPD)
          ldi     ZH,high(EXPD); HOLD2 % alternate code
          _DCR_M  ;-- large macro tbw
; PC no branch -81
          breq    PC+2
          rjmp    DIV0
          rjmp    ZERE0
;*
;*
;*
DIV5:     ror     A             ; WHICH NIBBLE DOES THIS QUOTIENT DIGIT GO IN?
          mov     A,ZH
;% JC DIV6  THIS ASSUMES THAT THE L COUNTER (2*DIGIT+1) WAS ODD.
          brcc    DIV6          ; % above replace this condition code
          ld      A,X
          swap A
;          lsl     A      ;RLC -- check cary rotate
 ;         lsl     A      ;RLC -- check cary rotate
  ;        lsl     A      ;RLC -- check cary rotate
   ;       lsl     A      ;RLC -- check cary rotate
          add     A,ZH
          st      X,A           ; STORE QUOTIENT
		  in r25,SREG
          adiw    XL,1
		  out SREG,r25
          rjmp    DIV7
;*   
DIV6:     st      X,A           ; STORE QUOTIENT
;*   
DIV7:     dec     ZL
; PC no branch -103
          breq    PC+2
          rjmp    DIV1
;*
;% alternate revised code
; CPI 5  THE LAST QUOTIENT DIGIT WILL BE USED FOR ROUNDING.
; JC DIV8  NO ROUNDING NECESSARY.
; LXI H,HOLD1+DIGIT-1  WHERE THE CURRENT QUOTIENT IS.
; CALL INCREMENT  ROUND BY ADDING 1 TO RIGHTMOST DIGIT.
;*
;DIV8 LDA XSIGND  PUT THE SIGN WHERE IT BELONGS.
; STA SIGND
;*   
          ldi     ZL,low(EXPD)
          ldi     ZH,high(EXPD); HOLD2  POINT TO EXPONENT OF QUOTIENT.
          pop     XH
          pop     XL
          rjmp    STORO
;*
;*
;start insert of file BSM#RAND  RANDOM NUMBER FUNCTION.
;*
;*
;*   THIS ROUTINE WILL GENERATE A RANDOM NUMBER
;*   BETWEEN 0 AND .99999999 IN FLOATING POINT FORMAT
;*
FRANDOM:  push    YL
          push    YH            ; SAVE ARGUMENT ADDRESS
          push    YL
          push    YH
          ld      A,Y
          or      A,A
          brne    RAN00
          ldi     ZL,low(RANLS+FPSIZ-1)
          ldi     ZH,high(RANLS+FPSIZ-1);          0x1CE0 LAST RANDOM NUMBER
; PC no jump 4200
          call    VCOPY         ; pgm_Vcopy COPY LAST TO STACK
          pop     YH
          pop     YL
          rjmp    RAN04
;*
;*
RAN00:    sbiw    YL,1
          ld      A,Y
          adiw    YL,1
          or      A,A
          breq    RAN01
          ldi     ZL,low((pgm_RANOS*2)+FPSIZ-1)
          ldi     ZH,high((pgm_RANOS*2)+FPSIZ-1);    0x1CE6 ORIG SEED
; PC no jump 4183
          call    pgm_VCOPY         ; pgm_Vcopy RESET TO ORIG
RAN01:    pop     YH
          pop     YL            ; ADDR OF ARG (ON STACK)
;*
RAN04:    mov     XH,YH         ; DE TO BC
          mov     XL,YL
          ld      A,X           ; GET EXPONENT
;% revised code
;CPI 80H  CHECK IF GREATER THAN .1
;JNC RAN20
;%
          mov     YH,A          ; SAVE EXPONENT
          ldi     YL,6          ; DIGI2
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
;*
RAN05:    ld      A,X           ; GET VALUE
          st      Z,A           ; PUT IN BUFFER
          sbiw    XL,1
          sbiw    ZL,1
          dec     YL
          brne    RAN05
;*
          st      Z,YL          ; PUT ZERO IN BUFFER
          mov     A,YH          ; % not in revised code
          cpi     A,0x80        ; % not in revised code
          brcc    R_1C84        ; % not in revised code
          ldi     XH,8          ; DIGIT+DIGIT % not in revised code
          subi    A,0x78        ;SUI; 0x78 % not in revised code
;% revised code
;MVI A,80H
;SUB D
;CPI DIGIT+DIGIT
;%
          brcs    RAN10         ; 
          com     A             ; % not in revised code
          ldi     c_tmp,9        ; ADI
          add     A,c_tmp       ; DIGIT+DIGIT+1 % not in revised code
          mov     XH,A          ; % not in revised code
;%MVI A,DIGIT+DIGIT  -- added line in revision
;*
RAN10:    mov     A,XH          ; % not in revised code
          add     A,A           ; FORM SHIFT COUNT
          add     A,A
          mov     XH,A
          rcall   RIGHT         ; FORM INTEGER VALUE
          pop     XH
          pop     XL            ; ARGUMENT ADDRESS
          push    XL
          push    XH            ; SAVE ADDRESS
          ldi     YL,low(0x0006)
          ldi     YH,high(0x0006); DIGI2
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)  ; BUFFER ADDRESS
;*
RAN15:    ld      A,Z           ; GET BUFFER VALUE
          st      X,A           ; PUT BACK INTO ARGUMENT
          st      Z,YH
          sbiw    XL,1
          sbiw    ZL,1
          dec     YL
          brne    RAN15
          st      Z,YH          ; % not in revised code
;% revised code
;*
;RAN20 MVI E,DIGI2  ZERO OUT THE SUM BUFFER.
; XRA A  MAKE A ZERO
; LXI H,EXP  END OF THE BUFFER.
;RAN21 MOV M,A  ZAP.
; DCX H
; DCR E
; JNZ RAN21  LOOP UNTIL BUFFER IS CLEARED.
;%
;*
;*   FORM A*X
;*
;%
R_1C84:   pop     YH
          pop     YL            ; RESTORE ADDRESS
          push    YL
          push    YH            ; SAVE ARGUMENT ADDRESS
          ldi     XL,1          ; SET HIGH LOW FLAG
          ldi     XH,8          ; DIGIT+DIGIT  DIGIT COUNT
          ldi     ZL,low(pgm_RANA*2)
          ldi     ZH,high(pgm_RANA*2) ; ADDRESS OF A
;*
;*
RAN30:    
          lpm
		  mov A,r0
;         ld      A,Z           ; GET NEXT DIGIT
          dec     XL            ; HIGH OR LOW DIGIT
          brne    RAN40         ; LOW DIGIT
          swap A
;          ror     A             ; GET HIGH DIGIT
 ;         ror     A
  ;        ror     A
   ;       ror     A
          sbiw    ZL,1          ; NOT NEXT BYTE YET
          ldi     XL,2          ; SET FOR LOW DIGIT
;*
RAN40:    andi    A,15          ; LOW 4 BITS
          adiw    ZL,1          ; NEXT BYTE
          push    ZL
          push    ZH            ; SAVE ADDRESS
          push    XL
          push    XH            ; SAVE COUNT
          mov     XL,A          ; DIGIT VALUE
;*
RAN50:    dec     XL            ; DECREMENT DIGIT VALUE
          brmi    RAN60         ; FINISHED
          push    YL
          push    YH            ; SAVE ARGUMENT ADDRESS
          in r25,SREG         
		  sbiw    YL,1          ; POINT TO LOW DIGIT
          sbiw    YL,1
		  out SREG,r25			; try it the old way
;ORA A  CLEAR THE CARRY. % revised addition
          rcall   S_ADD         ; MULTIPLY
          pop     YH
          pop     YL            ; RESTORE ARGUMENT ADDRESS
          rjmp    RAN50
;*
;*
RAN60:    pop     XH
          pop     XL            ; GET COUNT
          dec     XH
          breq    RAN70         ; FINISHED
          push    XL
          push    XH
          ldi     XH,4
          rcall   LEFT          ; SHIFT LEFT
          pop     XH
          pop     XL            ; RESTORE COUNT
          pop     ZH
          pop     ZL            ; GET DIGIT ADDRESS
          rjmp    RAN30         ; CONTINUE
;*   
RAN70:    pop     ZH
          pop     ZL            ; FIX STACK POINTER
;*
;*   FORM A*X+C
;*
          ldi     YL,low(RANC+DIGIT-1)
          ldi     YH,high(RANC+DIGIT-1); 0x32E9 C ADDRESS
;ORA A  CLEAR THE CARRY. % added code in revision
          rcall   S_ADD         ; AX+C
;*
;*   MAKE SURE SIGN IS POSITIVE, VALUE IS A FRACTION,
;*   AND CONVERT BACK TO FLOATING POINT FORMAT
;*   ...AND COPY TO SAVE FOR LAST RND NUMB
;*
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)  ; LOOK AT EXPONENT
          ldi     c_tmp,0x80
          st      Z,c_tmp       ; MAKE A FRACTION
          sbiw    ZL,1
          ldi     c_tmp,0
          st      Z,c_tmp       ; SIGN IS POSITIVE
;*   
          pop     XH
          pop     XL            ; RESULT ADDR
          ldi     ZL,low(RAN87)
          ldi     ZH,high(RAN87)
          push    ZL
          push    ZH
          push    XL
          push    XH
          rjmp    ADS4          ; NORMALIZE VALUE
;*
;*
RAN87:    ldi     XL,low(RANLS+FPSIZ-1)
          ldi     XH,high(RANLS+FPSIZ-1); 0x1CE0  ADDR OF LAST RAND NUMB
          rjmp    STORE         ; COPY VALUE AND ### EXIT ###
;*
;*
;% end insert of file BSM#RAND
;*
;*
;*
;* FETCH AND ALIGN ARGUMENTS FOR
;* ADDITION AND SUBTRACTION
;*
EXPCK:    ld      A,Y           ; FOR ADD & SUBTRACT ENTRY
          ld      c_tmp,Z
          sub     A,c_tmp       ; DIFFERENCE OF EXPS
          ldi     XL,0
          brcc    EXPC1
          inc     XL            ; INDICATE THAT THE ARGUMENTS WERE REVERSED.
          _XCHG
          com     A             ; SINCE THE ARGS WERE REVERSED, THE DIFFERENCE= - DIFF.
          inc     A
;*
EXPC1:    mov     XH,A          ; SAVE THE DIFFERENCE OF THE EXPONENTS.
          ld      A,Y           ; GET EXP OF LARGER #.
          sts     EXP,A         ; BECOMES THE EXP OF THE RESULT
;% more revised code
;*
; PUSH B  SAVE C, THE REVERSAL INDICATOR.
; PUSH D  SAVE ADDRESS OF EXP OF LARGER #.
; CALL LOAD  PUT THE SMALLER # IN BUF AND SIGN.  B UNCHANGED.
;*
          mov     A,XH          ; GET THE DIFFERENCE OF THE EXPS BACK.
          cpi     A,8           ; DIGIT+DIGIT+1  DIGIT+DIGIT IS THE MAX WE CAN SHIFT TO ALIGN
          brcs    EXPC2         ; EXPC2  DIFFERENCE WAS <= DIGIT+DIGIT.
;*
          ldi     A,8           ; DIGIT*2 % not in revised code
; % revised code
;*
;*  SINCE DIFFERENCE WAS GREATER THAN # OF DIGITS, NO ROUNDING
;*
; XRA A  MAKE A ZERO TO INDICATE
; STA RCTRL  (NONEXISTENT) ROUNDING DIGIT IN HIGH NIBBLE OF RDIGI
;*  RDIGI WAS SET TO 00 BY LOAD.
; MVI B,2*DIGIT*4  HOW MANY BITS TO SHIFT SMALLER #. CLEARS IT.
; JMP EXPC3  SKIP FINDING THE ROUNDING DIGIT.  GO SHIFT.
;*
EXPC2:    lsl     A      ;RLC -- check cary rotate; MULTIPLY EXP DIFF BY 4. CONVERT NIBBLES TO BITS.
          rol     A      ;RLC -- check cary rotate
          mov     XH,A          ; B NOW HAS # OF BITS TO SHIFT SMALLER # RIGHT.
          andi    A,4
; % revised code
;*
          sts     RCTRL,A       ; SET ROUNDING CONTROL, 1 IF ROUNDING DIGIT WILL BE IN
          push    XL
          push    XH            ; THE LOW NIBBLE OF RDIGI.
          push    YL
          push    YH
          rcall   LOAD
;% some of the above revised code is removed
;*
;*  THIS CODE FINDS WHERE THE DIGIT THAT GETS SHIFTED JUST OFF
;*  THE END IS, AND STORES IT IN RDIGI FOR LATER ROUNDING.
;*
          ldi     A,'0'         ; 8*DIGIT+16
          sub     A,XH
          cpi     A,'0'         ; 8*DIGIT+16
          breq    EXPC3         ; WILL BE NO SHIFTING, SO NO DIGITS LOST.
          andi    A,0xF8        ; CONVERT BITS REMAINING TO BYTES.
          lsr     A
          ror     A
          ror     A
          add     A,YL
          MOV     YL,A          ; MAKE THE ADDRESS OF THE DIGIT THAT WILL BE LOST.
          mov     A,YH
          ldi     c_tmp,0     ; ACI
          adc     A,c_tmp
          mov     YH,A
          ld      A,Y           ; ET ROUNDING DIGIT
          sts     RDIGI,A       ; SAVE
;*
;*
EXPC3:    rcall   RIGHT         ; ALIGN VALUES
          pop     YH
          pop     YL            ; DE RETURNED UNCHANGED FROM EXPCK.
          pop     XH
          pop     XL            ; C=1 IF ARGUMENTS WERE REVERSED.
          ret
;%
;% SETROUND moved here
;% INCREMENT added here
;% INCLOOP added here
;*
;*
;*   LOAD ARGUMENT INTO BUFFER
;*
LOAD:     ldi     YL,low(SIGN)
          ldi     YH,high(SIGN)
          ldi     XL,5          ; DIGIT+1
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
;*
LOAD1:    ld      A,Z
          st      Y,A
		  in r25,SREG
          sbiw    ZL,1
          sbiw    YL,1
		  out SREG,r25
          dec     XL
          brne    LOAD1
;*
          eor     A,A
          st      Y,A
 		  in r25,SREG
          sbiw    YL,1
		  out SREG,r25           ; probably could be more efficient
          st      Y,A
          sts     RDIGI,A       ; ZERO ROUNDING DIGIT
          ret
;*
;*
RIGH3:    mov     XH,A          ; SHIFT RIGHT ONE BYTE
          eor     A,A
RIGH4:    ld      YH,Z
          st      Z,A
          mov     A,YH
          in r25,SREG		; protect flags
          adiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    RIGH4
;*
;*
;*   SHIFT RIGHT B/4 DIGITS
;*
RIGHT:    ldi     XL,5          ; DIGIT+1
RIGH1:    ldi     ZL,low(BUF-1)
          ldi     ZH,high(BUF-1)
          mov     A,XH
          subi    A,8           ;SUI; CHECK IF BYTE CAN BE SHIFTED
          brcc    RIGH3
          dec     XH
          brpl    PC+2          ; RM
          ret
          clc	  ;or      A,A  -- this looks like clear cary
;*
RIGH2:    ld      A,Z
          ror     A
          st      Z,A
          in r25,SREG		; try protecting flags
          adiw    ZL,1
          out SREG,r25
		  dec     XL
          brne    RIGH2
          rjmp    RIGHT
;*
LEF3:     mov     XH,A          ; SHIFT LEFT ONE BYTE
          eor     A,A
LEF4:     ld      YH,Z
          st      Z,A
          mov     A,YH
          in r25,SREG		; protect flags
          sbiw    ZL,1
          out SREG,r25
          dec     XL
          brne    LEF4
;*
;*
;* SHIFT LEFT NUMBER OF DIGITS
;* IN B/4
;*
LEFT:     ldi     XL,5          ; DIGIT+1
LEFTA:    ldi     ZL,low(BUF+DIGIT-1)
          ldi     ZH,high(BUF+DIGIT-1); % alternative equ SIGN-1
;*   
LEF1:     mov     A,XH
          subi    A,8           ;SUI
          brcc    LEF3
          dec     XH
          brpl    PC+2          ; RM
          ret
          clc	  ;or      A,A  -- this looks like clear cary
;*
LEF2:     ld      A,Z
          rol     A
          st      Z,A
		  in r25,SREG		;% protect sregister
          sbiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    LEF2
          rjmp    LEFT
;*
; PC no branch -792
FMOVR:    brmi    PC+2
          rjmp    FMUL1
;*
;*
;*   SET FLAGS FOR OVERFLOW, AND UNDERFLOW
;*
OVER:     ldi     XL,low(0x4650)
          ldi     XH,high(0x4650)
; PC no jump 3297
          jmp     ERROR
;*
;*
;*
;*
;*     PERFORM UNSIGNED 16 BIT INTEGER MULTIPLY
;*     BC AND DE ARE ARGUMENTS TO MULTIPLY
;*     HL RETURNS RESULT
;*     OVERFLOW CAUSES OBERR
;*
;*
; simple AVR HW multiply version
IMUL:
	mul		YL, XL		; E * C
	movw	ZH:ZL, r1:r0
	mul		YH, XL		; D * C
	add		ZH, r0
	mul		XH, YH		; B * D
	add		ZH, r0
	ret

;*
;*  32 BIT BINARY MULTIPLY
;*  DE.HL=BC*DE
;*
; AVR HW mulitplier version
DIMUL:
;*	r19:r18:r17:r16 = r23:r22 * r21:r20
;*  AH  AL  ZH  ZL    YH  YL    XH  XL
mul16x16_32:
	clr		r2
	mul		YH, XH		; D * B
	movw	ARGH:ARGL, r1:r0
	mul		YL, XL		; E * C
	movw	ZH:ZL, r1:r0
	mul		YH, XL		; D * C
	add		ZH, r0
	adc		ARGL, r1
	adc		ARGH, r2
	mul		XH, YL		; B * E
	add		ZH, r0
	adc		ARGL, r1
	adc		ARGH, r2
	movw    YH:YL,ARGH:ARGL
	ret

;% end of file  BSM#MATH
;*
;*
;*     THE  FUNCTIONS
;*
;*
;% start of file  BSM#FUN1
;*
;*
;*
;*     UNARY AND BUILT IN FUNCTION ACTION ROUTINES
;*
;*  ALL EXPECT DE= # DE
;*
;*
;*     UNARY NEGATE
;*
FNEG:     ld      A,Y
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; ITS ZERO
          sbiw    YL,1          ; POINT TO SIGN
          ld      A,Y           ; GET SIGN
          ldi     c_tmp, 1
          eor     A,c_tmp       ; FLIP BIT 0
FNEG3:    st      Y,A
          ret
;*
;*
;*  RETURN ABSOLUTE VALUE OF ARG
;*
FABS:     sbiw    YL,1
          eor     A,A
          st      Y,A
          ret
;*
;*
;*  RETURN: -1 IFF ARG < 0
;*           0 IFF ARG = 0
;*           1 IFF ARG > 0
;*
FSGN:     ld      A,Y
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; IF ZERO RETURN ZERO
          sbiw    YL,1
          ld      A,Y
          or      A,A
          adiw    YL,1
          ldi     ZL,low(FPONE)		; conversion to odd pgm byte done
          ldi     ZH,high(FPONE)	; at complile time with equates
; PC no branch 3698
          brne    PC+3
          jmp    pgm_VCOPY         ; RETURN A 1 % pgm_
          ldi     ZL,low(FPNONE)	; conversion to odd pgm byte done
          ldi     ZH,high(FPNONE)	; conversion to odd pgm byte done
; PC no jump 3692
          jmp     pgm_VCOPY         ; RETURN -1 % pgm_
;*
;*
;*    CALL AN ASSEMBLY LANGUAGE ROUTINE
;*
; PC no jump 3826
FCALL:    call    EATL0
          rcall   PFIXE
          push    YL
          push    YH
; PC no jump 3856
          call    SCOMA
          brne    ACAL1
          rcall   PFIXE         ; COLLECT THE OPTIONAL ARGUMENT WHICH GOES IN D
;*
ACAL1:    ldi     ZL,low(ACAL2)
          ldi     ZH,high(ACAL2)
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          ijmp         ; PCHL
ACAL2:    rcall   FLOAT
; PC no jump 3810
ACAL3:    call    EATRP
          rjmp    EXPR2
;*
;*
;*  STRING LENGTH
;*
; PC no jump 3796
FLEN:     call    EATL0         ; (
          rcall   VAR
; PC no branch 3051
          breq    PC+3
          jmp    TYERR
          _XCHG
          rjmp    ACAL2         ; GO RETURN THE VALUE IN DE
;*
;*
;*  CONVERT CHARACTER TO ITS ASCII VALUE
;*
; PC no jump 3783
FASC:     call    EATL0
;*
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1    ; GET STRING EXPRESSION
          push    ZL
          push    ZH
          rcall   SEXPG
          pop     YH
          pop     YL
          _XCHG
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; POP STRING OFF
;*
          mov     A,YH
          or      A,YL
; PC no branch 3021
          brne    PC+3
          jmp    OBERR         ; STRING HAD 0 CHARS
          ld     ZL,Z          ; FIRST CHAR OF STRING
          ldi     ZH,0
          rjmp    ACAL2         ; FLOAT THE ASC CHARACTER AND RETURN
;*
;*
;*   INT FUNCTION ACTION ROUTINE
;*
FINT:     ld      A,Y
          subi    A,0x81           ;SUI; SUBTRACT EXPONENT BIAS
          ldi     XH,0xF4       ; -FPSIZ*2
          brmi    FINT3         ; LESS THAN ONE, MAKE ZERO
;*
;*  EXP => 1
;*
          subi    A,7           ;SUI; FPNIB-1
          brcs    PC+2          ; RNC
          ret                   ; RETURN IF ALREADY INTEGER
          mov     XH,A
          sbiw    YL,1
;*
FINT2:    sbiw    YL,1
          ld      A,Y
          andi    A,0xF0        ; HIGH NIBBLE GONE
          st      Y,A
          inc     XH
          brne    PC+2          ; RZ
          ret
;*
FINT3:    eor     A,A
          st      Y,A
          inc     XH
          brne    FINT2
          ret
;*
;*   INPUT FROM A I/O PORT
;*
FINP:     rcall   NFIX          ; PFIX
          ldi     A,0xDB        ; 333Q  OUT OPCODE
; PC no jump -3395
          call    OSET
; PC no jump 2516
          call    OBUF
          mov     ZL,A
AINP1:    ldi     ZH,0
          rjmp    FLOAT
;*
;*
;*   RETURNS THE AMOUNT OF FREE SPACE
;*
FFREE:    rcall   NFIX          ; PFIX
          mov     A,YH
          or      A,YL
; PC no branch 2963
          breq    PC+3
          jmp    OBERR
          lds     ZL,STA     ;LHLD
          lds     ZH,STA+1
          _XCHG
          lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          _DSUB
          rjmp    FLOAT         ; PUTS THE VALUE ON STACK AND RETURNS
;*
;*
;*  RETURNS THE VALUE AT MEMORY LOCATION
;*  POINTED TO BY DE
;*
FEXAM:    rcall   NFIX          ; PFIX
          _XCHG
          ld      ZL,Z
          rjmp    AINP1
;*
;*
;*    PRINTER HEAD POSITION
;*
FPOS:     rcall   NFIX          ; PFIX
          mov     A,YH
          or      A,YL
; PC no branch 2934
          breq    PC+3
          jmp    OBERR
          lds     A,PHEAD
          mov     ZL,A
          rjmp    AINP1
;*
;*
;*  RETURNS THE VALUE OF THE STRING
;*
; PC no jump 3669
FVAL:     call    EATL0
          ldi     ZL,low(0x0019)
          ldi     ZH,high(0x0019); WMAX-1  THIS IS THE MAXIMUM SIZED VALUE WE CAN USE
          sts     STRMX,ZL
          sts     STRMX+1,ZH    ; THIS MUST BE SET UP FOR STASS
          ldi     ZL,low(OBUF)
          ldi     ZH,high(OBUF) ; WE ARE GOING TO HAVE STASS STORE THE STRING I
          eor     A,A           ; TO SET Z AND PERMIT EXPRESSIONS AT STASS
          sec                   ; MAKE STASS THINK IT IS STORING INTO A SUBSTRING
          rcall   STASS         ; PUT CR AFTER LAST CHAR OF STRING IN CASE TERMIN
          ldi     A,13
          st      X,A
;*
;*  NOW STRING IS AT OBUF
;*
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
          ldi     YL,low(OBUF)
          ldi     YH,high(OBUF)
;*
;% this code is not in the PTDOS version
FVAL0:    ld      A,Y
          cpi     A,' '
          adiw    YL,1
          breq    FVAL0
          sbiw    YL,1
;% PT DOS version has FVAL0 here:
;*
          rcall   FPIN
          ldi     XL,low(0x494E)
          ldi     XH,high(0x494E); CAN'T CONVERT ERROR
; PC no branch 2933
          brcc    PC+3
          jmp    ERROR
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
; PC no jump 3507
          call    PSHAS
          rjmp    ACAL3
;*
;*
;*
;*     EOF FUNCTION
;*       RETURNS: 0  NO SUCH FILE OPEN
;*                1  LAST WAS OPEN FILE
;*                2  LAST WAS READ
;*                3  LAST WAS WRITE
;*                4  LAST WAS REWIND
;*                5  LAST WAS EOR (ONLY ON PTDOS)
;*                6  LAST WAS EOF
;*
FEOF:     rcall   NFIX          ; (PFIX) FIX THE ARGUMENT TO DE
; PC no jump -2902
          call    OPEEOF        ; GET FCB FOR THIS FILE
          ldi     ZL,0
; PC no branch -112
          breq    PC+2
          rjmp    AINP1         ; NO FCB, ZERO
          lds     A,CEOF
          mov     ZL,A
          rjmp    AINP1
;*
;*
;*
;*     TYP     RETURNS:
;*                1  NEXT DATA ITEM IS A NUMBER
;*                2  NEXT DATA ITEM IS A STRING
;*                3  THERE ARE NO MORE DATA ITEMS TO READ
;*
FTYP:     ld      A,Y
          or      A,A
; PC no branch 2852
          breq    PC+3
          jmp    OBERR         ; ARG MUST BE ZERO
; PC no jump 3495
          call    POPFP         ; GET RID OF IT
;*
          lds     ZL,RTXA     ;LHLD
          lds     ZH,RTXA+1     ; REMEMBER THE OLD RTXA
          push    ZL
          push    ZH
; PC no jump 3518
          call    XTXA
;*
; PC no jump 3621
          call    SCOMA         ; A COMMA?
          brcc    FTYP2         ; YES
;*
          ldi     XL,low(0x9F9F)
          ldi     XH,high(0x9F9F); DATARW*256+DATARW  FIND NEXT DATA STATEMENT
; PC no jump 3347
          call    LSTAT
          ldi     ZL,low(0x0003)
          ldi     ZH,high(0x0003); OUT OF DATA IF CARRY
          brcs    FTYP3
;*
FTYP2:    ldi     A,'"'
; PC no jump 3603
          call    SCANC
          ldi     ZL,low(1)
          ldi     ZH,high(1); IF NO QUOTE THEN A NUMBER
          brcs    FTYP3
          adiw    ZL,1          ; ELSE A STRING
;*   
; PC no jump 3485
FTYP3:    call    XTXA
          rcall   FLOAT         ; FLOAT(HL) TO TOS
          pop     ZH
          pop     ZL            ; OLD RTXA
          sts     RTXA,ZL
          sts     RTXA+1,ZH
          ret                   ; ALL DONE
;*
;*
;% end of BSM#FUN1
;*
;*
;*     THE  VARRIABLE  AND  CONSTANT  HANDLERS
;*
;*
;% start of BSM#VARC
;*
;*
;*     FIND VARIABLE (STRING OR SUBSTRING OR NUMBER OR MATRIX ELE
;*     NUMBER OR MATRIX ELEMENT CASE:
;*      HL POINTS TO VALUE
;*      Z CLEARED
;*     STRING OR SUBSTRING CASE:
;*      STRING MAX SIZE RETURNED IN STRMX GLOBAL
;*      LOGICAL LENGTH RETURNED IN STRLG AND DE
;*      BASE ADDRESS OF STRING RETURNED IN STRBA AND HL
;*      SYM.TAB. PTR TO LOGICAL LENGTH FIELD RETURNED IN BC
;*      CARRY SET IFF IT IS A SUBSTRING
;*      Z SET TO INDICATE THAT VAR FOUND A STRING OR SUBSTRING
;*
;*
; PC no jump 3273
VAR:      call    NAME
; PC no branch 2841
          brcc    PC+2
          rjmp    BSERR
;*
; PC no jump 3289
VAR0:     call    SNAME         ; CHECK IF STRING NAME
; PC no branch 104
          brcs    PC+2
          rjmp    VAR2          ; GO PROCESS STRING CASE
;*
          ld      A,Z           ; SNAME RETURNS TXA
          cpi     A,0xC8        ; LPARRW
          breq    VAR11         ; TEST IF SUBSCRIPTED
;*
;*     MUST BE SCALAR VARIABLE
;*
          rcall   STLK          ; RETURNS ENTRY ADDRESS IN HL
; PC no branch 85
          brcs    PC+2
          rjmp    VAR13
          ldi     YL,low(FPSIZ)
          ldi     YH,high(FPSIZ)
; PC no jump 3465
          call    ASTA1
          rjmp    VAR13
;*
;*     MUST BE SUBSCRIPTED
;*
VAR11:    adiw    ZL,1          ; EAT THE '('
          sts     TXA,ZL
          sts     TXA+1,ZH
          ldi     A,' '         ; MTYPE
          or      A,XL
          mov     XL,A          ; SET TYPE TO MATRIX
          rcall   STLK
          push    ZL
          push    ZH            ; SYMBOL TABLE PTR TO MAX SIZE FIELD
          brcc    PC+3    ; CC
          call   DIMM0         ; SYMTAB PTRDEFAULT DIMENSION MATRIX
          pop     ZH
          pop     ZL            ; SYMTAB PTR
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000)
          push    YL
          push    YH            ; INITIAL LOCATOR
          adiw    ZL,1          ; INCREMENT SYMTAB PTR TO FIRST DIMENSION TABLE ENTR
          adiw    ZL,1
          push    ZL
          push    ZH            ; DIMENSION TABLE PTR
;*
VAR12:    rcall   PFIXE
; PC no branch 2739
          brne    PC+2
          rjmp    OBERR         ; MUST BE > 0
          sbiw    YL,1
          pop     ZH
          pop     ZL            ; DIMTAB
; PC no jump 2970
          call    DCMP          ; SEE IF INDEX GREATER THAN DIMENSION SIZE
; PC no branch 2731
          brcs    PC+2
          rjmp    OBERR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; LOCATOR TO HL, DIMTAB TO STACK
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; INDEX PLUS LOCATOR TO HL
          _XCHG                 ; INDEX PLUS LOCATOR TO DE
          pop     ZH
          pop     ZL            ; DIMTAB TO HL
          adiw    ZL,1          ; ADVANCE TO
          adiw    ZL,1          ; NEXT DIMENSION SIZE (OR MARKER)
          ld      XL,Z          ; LOW ORDER BYTE OF NEXT DIM SIZE
          adiw    ZL,1
          ld      XH,Z          ; HIGH ORDER BYTE OF NEXT DIM SIZE
          push    ZL
          push    ZH            ; DIMTAB PTR TO LAST BYTE OF ENTRY
          mov     A,XH
          or      A,XL
          breq    VAR14         ; JUMP IF THIS SHOULD BE LAST DIMENSION
          rcall   IMUL          ; MULTIPLY NEXT DIMENSION TIMES CURRENT LOCATOR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; NEW LOCATOR TO STACK, DIMTAB PTR TO HL
          sbiw    ZL,1          ; MAKE POINT TO BEGINNING OF ENTRY IN DIMTAB
          push    ZL
          push    ZH
          ldi     XH,','
; PC no jump 3460
          call    EATC          ; SYNTAX ERROR IF NOT ANOTHER ARG
          rjmp    VAR12
;*
;*    COME HERE WHEN LOCATOR IS IN D AND DIMTAB PTR ON STACK
;*
; PC no jump 3452
VAR14:    call    EATRP
          pop     ZH
          pop     ZL            ; DIMENSION TABLE PTR (SHOULD POINT TO MARKER)
          adiw    ZL,1
          ldi     XL,6          ; FPSIZ  NUMBER OF TIMES FOR REPEATED ADD
;*
; PC no jump 3488
          call    RADD
;*
VAR13:    ori     A,1           ; CLEAR CARRY, SET NON-ZERO
          ldi     YL,low(0x0005)
          ldi     YH,high(0x0005); FPSIZ-1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          ret
;*
;*     STRING OR SUBSTRING CASE
;*
VAR2:     rcall   STLK
          push    ZL
          push    ZH            ; SYMTAB PTR TO MAX
          ldi     YL,low(10)
          ldi     YH,high(10)
          brcc    PC+3   ; CC
          call   DIMST         ; CREATE DEFAULT STRING IF DOESN'T EXIST
          pop     ZH
          pop     ZL
; PC no jump 3025
          _DLOAD         ; SYMTAB PTR
          _XCHG
          sts     STRMX,ZL
          sts     STRMX+1,ZH
          _XCHG
          push    ZL
          push    ZH            ; SYMTAB PTR TO LG
; PC no jump 3016
          _DLOAD
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          _XCHG
          sts     STRLG,ZL
          sts     STRLG+1,ZH
;*
          ldi     A,0xC8        ; LPARRW
; PC no jump 3434
          call    SCANC
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
; PC no branch 118
          brcs    PC+2
          rjmp    VAR6          ; GO RETURN VALUES FOR STRING VARIABLE CASE
;*
          lds     ZL,STRMX     ;LHLD
          lds     ZH,STRMX+1
          push    ZL
          push    ZH
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
          push    ZL
          push    ZH
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
          push    ZL
          push    ZH
          rcall   PFIXE
          pop     ZH
          pop     ZL
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          pop     ZH
          pop     ZL
          sts     STRLG,ZL
          sts     STRLG+1,ZH
          pop     ZH
          pop     ZL
          sts     STRMX,ZL
          sts     STRMX+1,ZH
; PC no branch 2616
          brne    PC+2
          rjmp    OBERR
;*
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
; PC no jump 2840
          _HDCMP
; PC no branch 2607
          brcc    PC+2
          rjmp    OBERR
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
          sbiw    ZL,1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          push    YL
          push    YH            ; FIRST INDEX
;*
; PC no jump 3377
          call    SCOMA
          brcs    VAR5
;*
          lds     ZL,STRMX     ;LHLD
          lds     ZH,STRMX+1
          push    ZL
          push    ZH
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
          push    ZL
          push    ZH
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
          push    ZL
          push    ZH
          rcall   PFIXE
          pop     ZH
          pop     ZL
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          pop     ZH
          pop     ZL
          sts     STRLG,ZL
          sts     STRLG+1,ZH
          pop     ZH
          pop     ZL
          sts     STRMX,ZL
          sts     STRMX+1,ZH
; PC no branch 2562
          brne    PC+2
          rjmp    OBERR
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
; PC no jump 2786
          _HDCMP
; PC no branch 2553
          brcc    PC+2
          rjmp    OBERR
          _XCHG
          sts     STRLG,ZL
          sts     STRLG+1,ZH
;*
; PC no jump 3299
VAR5:     call    EATRP
          pop     YH
          pop     YL            ; FIRST INDEX
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
          _DSUB
; PC no branch 2536
          brpl    PC+2
          rjmp    OBERR         ; WE HAVE SUBTRACTED FIRST INDEX FROM SECOND
          adiw    ZL,1          ; ADD 1 TO GET EXACT LENGTH
          sts     STRLG,ZL
          sts     STRLG+1,ZH
          sts     STRMX,ZL
          sts     STRMX+1,ZH
          sec
;*
VAR6:     pop     XH
          pop     XL
          lds     ZL,STRLG     ;LHLD
          lds     ZH,STRLG+1
          _XCHG
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
          ldi     A,1
          dec     A             ; SET Z FLAG WITHOUT AFFECTING CARRY
          ret
;*
;*
;*      SYMBOL TABLE LOOKUP
;*      BC CONTAIN NAME AND TYPE
;*      IF NOT FOUND THEN CREATE ENTRY AND SET CARRY
;*      HL HAS ADDRESS AFTER POINTER ON RETURN
;*      IF ENTRY CREATED THEN STNBA GETS BASE ADDR OF NEW ENTRY
;*      AND STNPTR GETS ADDR OF POINTER TO NEW ENTRY
;*
;*
STLK:     mov     A,XH          ; COMPUTE CHAIN POINTER ADDR
          subi    A,'A'           ;SUI
          add     A,A
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
          adiw    ZL,1
          add     A,ZL
          mov     ZL,A
          brcc    STLK1
          inc     ZH            ; HL CONTAINS CHAIN POINTER
;*
;*   LOOKUP LOOP, FOLLOW CHAIN
;*
STLK1:    ld      YL,Z          ; FOR H
          adiw    ZL,1
          ld      YH,Z          ; THEN L
          mov     A,YH
          sbiw    ZL,1          ; DE POINTS TO NEXT, HL POINTS TO THIS
          _XCHG                 ; HL POINTS TO NEXT, DE POINTS TO THIS
          or      A,ZL
          breq    STLK2         ; JMP IF END OF CHAIN (POINTER TO NEXT IS ZERO)
;* 
          ld      A,Z           ; GET 2ND CHAR (0 MEANS NO 2ND CHAR)
          adiw    ZL,1          ; PASS 2ND CHAR
;% ANI -1-COMVD  DONT CARE ABOUT COMMON FLAG AT THIS POINT
          cp      A,XL          ; SAME?
          brne    STLK1         ; JMP IF ENTRY NOT THE ONE
;*
          adiw    ZL,1          ; PASS POINTER TO NEXT SO HL WILL...
          adiw    ZL,1          ; POINT TO THE DATA IN THIS VAR
          ret                   ; CARRY IS CLEAR
;*
;*       CREATE NEW ENTRY
;*
STLK2:    push    YL
          push    YH            ; POINTER TO LAST ENTRY
          ldi     ZL,low(0x0003)
          ldi     ZH,high(0x0003)
; PC no jump 3169
          call    ASTAB         ; MAKE ROOM FOR NEW ENTRY
          sts     STNBA,ZL
          sts     STNBA+1,ZH    ; ADDRESS OF NEW ENTRY (FOR UNDF AT ERROR)
          _XCHG                 ; TO DE
          pop     ZH
          pop     ZL            ; ADDR OF LAST ENTRY (ITS FORE POINTER)
          sts     STNPTR,ZL
          sts     STNPTR+1,ZH   ; ADDRESS OF LAST FORE POINTER UPDATED (ERROR PROC)
; PC no jump 2814
          _DSTOR         ; MAKE LAST ENTRY POINT TO NEW ONE
          _XCHG                 ; DE (ADDR OF NEW ENTRY) TO HL
          st      Z,XL          ; SAVE 2ND CHAR OF NAME
          adiw    ZL,1          ; PASS 2ND CHAR
          adiw    ZL,1          ; PASS NEW ENTRY'S FORE POINTER (ZERO NOW)
          adiw    ZL,1
          ldi     A,1
          sts     UNDEF,A       ; IN CASE OF ERROR, SO ERROR LOGIC CAN UN-DEFINE
          sec                   ; A VAR WAS JUST DEFINED (C=1)
          ret
;*
;*
;*     GETS FP CONSTANT FROM TEXT
;*     PUSHES VALUE ON ARG STACK
;*     AND SETS ARGF - SETS CARRY IFF NOT FOUND
;*
;*
CONST:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          _XCHG                 ; POINTER TO CONSTANT IS TXA (INTO DE)
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
          rcall   FPIN
          brcc    PC+2          ; RC
          ret
          sbiw    YL,1
          _XCHG
          sts     TXA,ZL
          sts     TXA+1,ZH      ; NOW POINTS TO TERMINATOR
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
; PC no jump 3060
          jmp     PSHAS         ; SETS ARGF, CLEARS CARRY AND RETURNS
;*
;*
;*
;*       GET STRING CONSTANT FROM TEXT
;*     ACC=0  MEANS " IS TERMINATOR
;*     ACC#0  MEANS ACC IS TERMINATOR
;*     RETURNS STRING DESCRIPTION A LA VAR
;*     SETS CARRY WITH TXA UNCHANGED IF NO STRING CONST FOUND
;*
;*
SCONS:    cpi     A,'"'
          breq    SCON1
          or      A,A
          brne    SCON2
          ldi     A,'"'
; PC no jump 3201
SCON1:    call    SCANC
          brcc    PC+2          ; RC
          ret
;*
SCON2:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          sts     STRBA,ZL
          sts     STRBA+1,ZH
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000)
          mov     XH,A
;*
SCON3:    ld      A,Z
          adiw    ZL,1
          cp      A,XH
          breq    SCON4
          cpi     A,13
          adiw    YL,1
          brne    SCON3
; PC no jump 2437
          jmp     BSERR
;*
SCON4:    sts     TXA,ZL
          sts     TXA+1,ZH
          lds     ZL,STRBA     ;LHLD
          lds     ZH,STRBA+1
          ret
;*
;*    CERTAIN STATEMNTS MAY NOT APPEAR WITHIN IF STATEMENTS, OR
;*    MAY NOT BE EXECUTED IN THE DIRECT MODE
;*
DIRT0:    lds     A,IFTERM
          cpi     A,13
; PC no branch -4022
          breq    PC+3
          jmp    CSERR
;*FALL THRU TO DIRT
;*
;*     DIRECT STATEMENT CHECKING ROUTINES
;*
DIRT:     lds     A,DIRF
          or      A,A
          brne    PC+2          ; RZ
          ret
          ldi     XL,low(0x4449)
          ldi     XH,high(0x4449)
; PC no jump 2414
          jmp     ERROR
;*
;*     SUBTRACT DE FROM HL
;*
;DSUB:
;	sub ZL,YL
;	sbc ZH,YH
;ret
;% end of file BSM#VARC
;*
;*
;*     THE  CONVERSION  ROUTINES
;*
;*
;% start of file BSMCVT1
;*
;*     FLOAT THE VALUE IN HL TO TOP OF STACK
;*
FLOAT:    ldi     YL,low(OBUF)
          ldi     YH,high(OBUF)
          rcall   CNS
          ldi     A,13
          st      Y,A
          ldi     YL,low(OBUF)
          ldi     YH,high(OBUF)
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
          rcall   FPIN
          ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK)
; PC no jump 2951
          jmp     PSHAS
;*
;*     FIX FLOATING TO INTEGER
;*     RETURN INTEGER VALUE IN DE
;*     FP VALUE FROM TOP OF ARG STACK, POP ARG STACK
;*     ZERO FLAG SET  IF ZERO RESULT
;*
; PC no jump -2798
PFIXE:    call    EXPRB         ; CALL NFIXE->EXPRB
;*
;% NFIXE CALL EXPRB  ENTRY POINT THAT GETS EXPRESSION FROM TEXT
;*
; PC no jump 2935
NFIX:     call    TOPFP
          push    ZL
          push    ZH
          rcall   FINT
; PC no jump 2955
          call    POPFP         ; POP TOP FP TO FPSINK
          pop     ZH
          pop     ZL
          ld      A,Z           ; EXPONENT
          ld      XL,Z          ; TWICE
          sbiw    ZL,1          ; POINT TO SIGN
          ld      XH,Z          ; GET IT
          ldi     YL,low(0xFFFB)
          ldi     YH,high(0xFFFB); -FPSIZ+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; H NOW POINTS TO MSB-1
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000); IN CASE OF ZERO
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; RETURN IF ZERO
          mov     A,XH
          or      A,A
; PC no branch 2290
          breq    PC+2
          rjmp    OBERR
          dec     XL            ; SET UP FOR LOOP
PFIX1:    adiw    ZL,1
          ld      A,Z
		  swap A
;          lsr     A      ;RRC -- check cary rotate
 ;         lsr     A      ;RRC -- check cary rotate
  ;        lsr     A      ;RRC -- check cary rotate
   ;       lsr     A      ;RRC -- check cary rotate
          push    ZL
          push    ZH
          rcall   MUL10
          pop     ZH
          pop     ZL
; PC no branch 2275
          brcc    PC+2
          rjmp    OBERR
          dec     XL
          brpl    PFIX2
          ld      A,Z
          push    ZL
          push    ZH
          rcall   MUL10
          pop     ZH
          pop     ZL
; PC no branch 2262
          brcc    PC+2
          rjmp    OBERR
          dec     XL
          brmi    PFIX1
PFIX2:    mov     A,YH
          or      A,YL
          ret
;*
;*     GET INTEGER FROM TXA
;*     SET CARRY IF NOT FOUND
;*     RETURNS INTEGER VALUE IN HL
;*     RETURNS TERMINATOR IN ACC
;*
INTGER:   lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; DO THIS TO SKIP OVER LEADING BLANKS
          sts     STRPT,ZL
          sts     STRPT+1,ZH
          rcall   GNC1
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcc    PC+2          ; RC
          ret
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000)
;*
INTG1:    rcall   MUL10
; PC no branch 2235
          brcc    PC+2
          rjmp    OBERR
          rcall   GNC
          brcs    INTG1         ; LOOP IF CHAR IS ANOTHER DIGIT
          lds     ZL,STRPT     ;LHLD
          lds     ZH,STRPT+1
          sbiw    ZL,1          ; LEAVE TXA POINTING TO TERMINATOR (BLANK OR WHATEVE
          sts     TXA,ZL
          sts     TXA+1,ZH      ; TXA WILL POINT 1 AFTER LAST DIGIT
          _XCHG
          ret                   ; CARRY IS CLEAR
;*
;*     GET CODED LINE NUMBER FOR GOTO, GOSUB OR ON STATEMENT
;*
LNUM:     ldi     A,0x96        ; LNRW
; PC no jump 2999
          call    SCANC
          brcc    PC+2          ; RC
          ret                   ; NO LN
;*
          ld      YL,Z
          adiw    ZL,1          ; HIGH ORDER
          ld      YH,Z
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH      ; TO HL, TXA TO DE
          _XCHG
;*
          clc     ;or      A,A           ; TO CLEAR CARRY
          ret
;*
;*     CONVERT INTEGER TO STRING
;*     DE CONTAINS ADDRESS OF STRING, RETURN UPDATED VALUE IN DE
;*     HL CONTAINS VALUE TO CONVERT
;*
CNS:      eor     A,A           ; SET FOR NO LEADING  BLANKS
CLNS:     ldi     XL,low(-10000)
          ldi     XH,high(-10000)
;*
;*  AT THIS POINT:
;*   A = 0    FOR NO LEADING CHARACTERS
;*   A = 1    FOR LEADING ZEROS
;*   A = ' '  FOR LEADING BLANKS
;*
          rcall   RSUBX
          ldi     XL,low(-1000)
          ldi     XH,high(-1000)
          rcall   RSUBX
CNS3:     ldi     XL,low(-100)
          ldi     XH,high(-100) ; ENTRY POINT FROM FPOUT FOR EXPONENT
          rcall   RSUBX
CNS1:     ldi     XL,low(-10)
          ldi     XH,high(-10)
          rcall   RSUBX
          ldi     XL,low(-1)
          ldi     XH,high(-1)
          mov     A,XH          ; FORCE A FINAL ZERO
          rcall   RSUBX
          ret
;*
;*     TAKES VALUE IN HL
;*     SUB MINUS NUMBER IN BE THE MOST POSSIBLE TIMES
;*     PUT VALUE ON STRING AT DE
;*     IF A=0 THEN DONT PUT BLANK ON STRING
;*     RETURN NON-ZERO IN A IF SOMETHING PUT ON STRING
;*
RSUBX:    push    YL
          push    YH
          ldi     YH,0xFF
;*
RSUB9:    inc     YH
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          brcs    RSUB9
;*
;*  SUBTRACT OVERFLOWED, ADD BACK LAST VALUE TO MAKE POSITIVE
;*
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          mov     A,XH
          com     A
          mov     XH,A
          mov     A,XL
          com     A
          mov     XL,A
          adiw    XL,1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
;*   
          mov     XH,YH
          pop     YH
          pop     YL
          or      A,XH          ; A GETS 0 IFF A IS 0 AND B IS 0
          brne    PC+2          ; RZ
          ret
;*   
          subi    A,' '           ;SUI
          or      A,XH
          brne    RSUB2         ; IF A=' ' AND B=0 THEN LEADING BLANKS
          ldi     A,' '
          rjmp    RSUB3
;*   
;*   
RSUB2:    ldi     A,'0'
          add     A,XH
RSUB3:    st      Y,A
          adiw    YL,1
          ret
;*
;*     TAKE NEXT DIGIT IN A (MASK TO 17Q)
;*     ACCUMULATE TO VALUE IN DE
;*     PRESERVES ALL BUT A, DE
;*
MUL10:    mov     ZH,YH
          mov     ZL,YL
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH         ; GIVING 2X
          brcc    PC+2          ; RC
          ret
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH         ; GIVING 4X
          brcc    PC+2          ; RC
          ret
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; GIVING 5X
          brcc    PC+2          ; RC
          ret
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH         ; GIVING 10X
          brcc    PC+2          ; RC
          ret
          _XCHG
          andi    A,15
          add     A,YL
          MOV     YL,A
          mov     A,YH
          ldi     c_tmp,0     ; ACI
          adc     A,c_tmp       ; PROPOGATE THE CARRY
          mov     YH,A
          ret
;*
;*     TAKES ADDRESS OF FP VALUE IN  HL
;*     HL PRESERVED
;*     SETS Z IF THE VALUE IS INTEGRAL
;*
ITEST:    push    ZL
          push    ZH
          push    ZL
          push    ZH
          ldi     YL,low(FPSINK)
          ldi     YH,high(FPSINK)
          push    YL
          push    YH
          push    YL
          push    YH
; PC no jump 2710
          call    VCOPY
          pop     YH
          pop     YL
          rcall   FINT          ; CONVERT THE FPSINK ONE TO INTEGER  
          pop     ZH
          pop     ZL            ; FPSINK
          pop     YH
          pop     YL            ; ORIGINAL VALUE
; PC no jump -2258
          call    RELOP         ; WILL RETURN WITH Z SET OR CLEAR
          pop     ZH
          pop     ZL
          ret
;*
;*
;% end of file BSM#CVT1
;*
; IF M60K  MUST SPLIT THE PROGRAM AT 0000
; IF $+1/1000H-0FH  IF $ IS > FFFE (SORT OF) THEN ERROR.
; AN ERROR!  PROGRAM RUNS PAST FFFE!
; ENDF
; ORG 100H
; ENDF
;*
;% start of file BSM#CVT2
;*
;*   
;*     OUTPUT FLOATING POINT VALUE POINTED TO BY HL
;*     RETURN POINTER TO FIRST CHAR OF RESULT STRING IN HL
;*     AND SIZE OF RESULT STRING IN B
;*   
;* 
FPOUT:    push    ZL
          push    ZH
          ldi     A,4
          sts     COMCNT,A      ; IN CASE COMMAS ARE REQUIRED
          eor     A,A
          sts     EFRMF,A       ; THIS VALUE SAYS NOT
          lds     A,CFORM       ; LOAD FLOATING POINT FORMAT
          cpi     A,'I'         ; INTEGER
; PC no branch 520
          brne    PC+2
          rjmp    IFORM
          cpi     A,'#'
; PC no branch 464
          brne    PC+2
          rjmp    FREE
          cpi     A,'E'
; PC no branch 535
          brne    PC+2
          rjmp    EFORM
;*   
;*  MUST BE 'F' FORMAT
FF:       lds     A,CWIDTH
          rcall   FFORM
;*   
;*  NOW SEE ABOUT OPTIONAL '+' AND '$'
FP1:      pop     ZH
          pop     ZL            ; POINTER TO VALUE
          lds     A,COPT
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; OPTION WORD WILL BE NEEDED LATER
          andi    A,' '
          breq    FP2           ; JUMP IF '+' NOT DESIRED
          sbiw    ZL,1          ; POINT TO SIGN
          ld      A,Z           ; SIGN BYTE
          or      A,A
          ldi     A,'+'
          brne    PC+2          ; CZ
          rcall   LFP
;*
FP2:      pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; OPTION WORD
          ror     A             ; MOV RIGHT BIT TO CARRY
          ldi     A,'$'
          brcc    PC+2          ; CC
          rcall   LFP
          lds     ZL,RESTA     ;LHLD
          lds     ZH,RESTA+1
;*   
;*  NOW SEE IF WANT TO DO SPECIAL FREE FORMAT STUFF
;*
          lds     A,CFORM
          cpi     A,'#'
          brne    FP4
;*
FP3:      adiw    ZL,1
          ld      A,Z
          cpi     A,' '
          breq    FP3
          cpi     A,'+'
          breq    FP4
          cpi     A,'-'
          breq    FP4
          cpi     A,'$'
          breq    FP4
          sbiw    ZL,1          ; ADD ONE LEADING SPACE BEFORE THE APPEARENT DIGIT
;*   
;*  NOW COUNT CHARACTERS IN RESULT SRING TO B REGISTER
FP4:      ldi     XH,0
          mov     YH,ZH
          mov     YL,ZL
;*   
FP41:     ld      A,Y
          cpi     A,'"'
          brne    PC+2          ; RZ
          ret
          inc     XH
          adiw    YL,1
          rjmp    FP41
;*   
;*   
;*   
;*   
;*   F FORMAT SUBROUTINE
;*   EXPECTS WIDTH IN ACC, VALUE PTR IN HL
;*   SET CARRY ON FAILURE TO FIT INTO FIELD
;*   RETURN PTR TO FIRST CHAR OF RESULT IN HL
;*   ALSO, EPOINT POINTS TO LAST NON-BLANK DIGIT OF RESULT (
;* 
FFORM:    sts     VALUE,ZL
          sts     VALUE+1,ZH
          mov     XH,A          ; REMEMBER WIDTH
;*   
;*  INIT THE OUTPUT BUFFER
          ldi     ZL,low(EOBUF+1)
          ldi     ZH,high(EOBUF+1)
          ldi     c_tmp,'"'
          st      Z,c_tmp
          lds     A,CFRACT
          or      A,A
;*   
FF1:      breq    FF2
          sbiw    ZL,1
          ldi     c_tmp,'0'
          st      Z,c_tmp
          dec     XH
          dec     A
          rjmp    FF1
;* 
FF2:      sbiw    ZL,1
          dec     XH
          ldi     c_tmp,'.'
          st      Z,c_tmp
          sts     DPOINT,ZL
          sts     DPOINT+1,ZH   ; THIS VALUE USED BY  E EFORMAT LOGIC TO CHECK
          sts     EPOINT,ZL
          sts     EPOINT+1,ZH
          sts     RESTP,ZL
          sts     RESTP+1,ZH    ; THIS VALUE USED BY LFP
;* 
FF3:      cp      A,XH          ; A IS ZERO FROM FF1
          breq    FF4
          dec     XH
          sbiw    ZL,1
          ldi     c_tmp,' '
          st      Z,c_tmp
          rjmp    FF3
;* 
FF4:      sts     RESTA,ZL
          sts     RESTA+1,ZH    ; RESULT ADDRESS
          sbiw    ZL,1
          ldi     c_tmp,'!'
          st      Z,c_tmp       ; MARKER FOR BEGINNING OF VALUE (FOR LFP)
          lds     ZL,VALUE     ;LHLD
          lds     ZH,VALUE+1
          ld      A,Z
          or      A,A
; PC no branch 104
          brne    PC+2
          rjmp    FF82          ; GO REMOVE TRAILING ZEROES
          subi    A,0x81           ;SUI; UNBIAS
; PC no branch 197
          brcc    PC+2
          rjmp    FF9           ; D-CASE (ENTIRE VALUE TO RIGHT OF DPOINT)
          inc     A             ; CORRECT FOR BIAS GLITCH (SHOULD HAVE SUBTRACTED 10)
          mov     XH,A          ; REMEMBER EXPONENT VALUE
          mov     XL,A
          subi    A,8           ;SUI
          brcs    FF7           ; C-CASE (VALUE OVERLAPS THE DP)
;*   
;*  ZERO FILL BETWEEN DP AND LSB OF VALUE
          breq    FF6
          mov     XL,A
;*
FF5:      ldi     A,'0'
          rcall   LFPD
          dec     XL
          brne    FF5
;*
FF6:      ldi     XH,8
          mov     XL,XH         ; THIS VALUE CAUSES ONLY ZEROES TO BE PUT TO RIGHT
;*   
;*  B HAS INDEX OF NEXT DIGIT TO BE WRITTEN TO THE LEFT
FF7:      inc     XL            ; ADVANCE C FOR USE LATER (INDEX OF FIRST DIGIT TO
;* 
FF71:     rcall   GFD           ; GET DIGIT
          rcall   LFPD
          dec     XH
          brne    FF71
;*   
;*    C HAS DIGIT INDEX IN VALUE
;*    LOOP TERMINATES WHEN RFPD EXITS TO FF81 OR FF82
;* 
FF80:     mov     XH,XL
;* 
FF8:      rcall   GFD           ; GET THE DIGIT
          rcall   RFPD
          inc     XH
          rjmp    FF8
;*   
;*    COME HERE FROM RFPD IF ROUNDING REQUIRED
;*    H HAS POINTER TO THE '"' CHARACTER
;*  
FF81:     sbiw    ZL,1
          ld      A,Z
          cpi     A,'.'
          breq    FF81
          cpi     A,','
          breq    FF81
          cpi     A,'"'
          brcs    FF8111
;*   
;*  MUST BE A DIGIT, SO INCREMENT IT
          inc     A
          st      Z,A
          cpi     A,':'
          brne    FF82          ; CARRY NO PROPOGATED
          ldi     c_tmp,'0'
          st      Z,c_tmp
          rjmp    FF81
;*   
;*  COME HERE IF BLANK OR '!' ENCOUNTERED WHEN PROPOGATING CAR
FF8111:   lds     A,EFRMF       ; THIS FLAG FOR E FORMAT
          or      A,A
          breq    FF812         ; JUMP IF NOT E FORMAT
;*   
;*  CONVERT TO 1.00 AND BUMP EXPONENT
          adiw    ZL,1          ; POINT TO CHARACTER PRECEDING DP
          ldi     c_tmp,'1'
          st      Z,c_tmp       ; BUMP EXPONENT SET CARRY FOR OVERFLOW
          ldi     c_tmp,1        ; ADI
          add     A,c_tmp
; PC no branch 1846
          brcc    PC+2
          rjmp    FOERR         ; EXPONENT OVERFLOW (COEFFICIENT SIGN HAS BEEN RE
          st      Y,A
          rjmp    FF82          ; DONE ROUNDING
;*   
;*  HERE IT WAS NOT E FORMAT
FF812:    ldi     A,'1'
          rcall   LFPD
;*   
;*  REMOVE TRAILING ZEROES IF REQUIRED
FF82:     ldi     ZL,low(EOBUF)
          ldi     ZH,high(EOBUF); EOBUF-1
          lds     A,CFORM       ; TRIALING ZEROES REMOVED IN FREE FORMAT
          cpi     A,'#'
          breq    FF83
          lds     A,COPT
          or      A,A
          brpl    FF84          ; JUMP IF TRAILING ZEROES WANTED
;*   
FF83:     ld      A,Z
          cpi     A,'0'
          brne    FF831
          ldi     c_tmp,' '
          st      Z,c_tmp
          sbiw    ZL,1
          rjmp    FF83
;*   
FF831:    lds     A,CFORM       ; IF FREE FORMAT, MOVE UP THE "
          cpi     A,'#'
          brne    FF84
          adiw    ZL,1
          ldi     c_tmp,'"'
          st      Z,c_tmp
          sbiw    ZL,1
;*   
;*  CHECK FOR SPECIAL CASE OF ZERO VALUE
FF84:     lds     A,CFORM       ; CK FOR FREE FORM
          cpi     A,'#'
          brne    FF84B
          sts     EPOINT,ZL
          sts     EPOINT+1,ZH   ; FOR FREE FORMAT
;*
FF84B:    ld      A,Z           ; HL HAS POINTER TO LAST NON-BLANK CHARACTER
          cpi     A,'.'
          brne    FF85
          sbiw    ZL,1
          ld      A,Z
          cpi     A,' '
          breq    FF841
          cpi     A,'!'
          brne    FF85
          adiw    ZL,1
          adiw    ZL,1
          ldi     c_tmp,'0'
          st      Z,c_tmp
          rjmp    FF85
;*
FF841:    ldi     A,'0'
          rcall   LFPD
;*   
;*  NOW PUT ON '-' IF NEEDED
FF85:     lds     ZL,VALUE     ;LHLD
          lds     ZH,VALUE+1    ; VALUE
          sbiw    ZL,1
          ld      A,Z
          or      A,A
          ldi     A,'-'
          breq    PC+2          ; CNZ
          rcall   LFP           ; PUT ON '-' IF NEGATIVE VALUE
          lds     ZL,RESTA     ;LHLD
          lds     ZH,RESTA+1
          ret
;*   
;*    D CASE
;*    WRITE ZEROES AFTER DPOINT
;*   
FF9:      com     A             ; THE ACC HAD 1'S COMPLEMENT OF EXPONENT MAGNITUDE
          or      A,A           ; SET Z IN CASE OF ZERO
          mov     XH,A
          ldi     XL,1          ; NEEDED AT FF8 (DIGIT TO PRINT NEXT
;* 
; PC no branch -170
FF91:     brne    PC+2
          rjmp    FF80
          ldi     A,'0'
          rcall   RFPD          ; WILL EXIT IF FIELD FILLS UP
          dec     XH
          rjmp    FF91
;*   
;*    ADD CHARACTER TO LEFT OF INTEGER (FOR FPOUT)
;*    SET CARRY IF OUT OF ROOM
;*    CHARACTER EXPECTED IN ACC
;*    PRESERVE BC
;*   
LFPD:     push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; ENTER HERE IF DIGIT
          lds     A,COPT
          andi    A,'@'         ; TEST IF COMMAS REQUIRED
          breq    LFP1          ; NOT DESIRED
          ldi     ZL,low(COMCNT)
          ldi     ZH,high(COMCNT)
          _DCR_M  ;-- large macro tbw
          brne    LFP1
;*   
;*  COMMA MUST BE WRITTEN
          ldi     c_tmp,3
          st      Z,c_tmp
          ldi     A,','
          rcall   LFP
;*
LFP1:     pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
;* 
LFP:      lds     ZL,RESTP     ;LHLD
          lds     ZH,RESTP+1
          sbiw    ZL,1
          mov     YH,A
          ld      A,Z
          cpi     A,'!'
; PC no branch 1687
          brne    PC+2
          rjmp    FOERR
          sts     RESTP,ZL
          sts     RESTP+1,ZH
          st      Z,YH
          ret
;*   
;*    ADD  DIGIT TO RIGHT END OF FP STRING (FPOUT)
;*    CHARACTER IN ACC
;*    IF CHARACTER BEYOND FIELD, GO TO FF81 IF ROUNDING
;*    REQUIRED, ELSE GOTO FF83
;*    PRESERVE BC
;*   
RFPD:     lds     ZL,EPOINT     ;LHLD
          lds     ZH,EPOINT+1
          adiw    ZL,1
          mov     YH,A
          ld      A,Z
          cpi     A,'"'
          breq    RFPD1
          sts     EPOINT,ZL
          sts     EPOINT+1,ZH
          st      Z,YH
          ret
;*   
;*  HERE ENCOUNTERED END OF FIELD
RFPD1:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; CLEAN OFF RETURN LINK
          mov     A,YH
          cpi     A,'5'
; PC no branch -178
          brcc    PC+2
          rjmp    FF82
          rjmp    FF81
;*   
;*    GET BTH DIGIT FROM FP VALUE
;*    PRESERVE BC, RETURN '0' IF B>PREC
;*
GFD:      ldi     A,8           ; FPNIB
          mov     YH,A          ; SAVE IT IN D
          cp      A,XH          ; CHECK FOR INDEX > PREC
          ldi     A,'0'
          brcc    PC+2          ; RC
          ret
          mov     A,XH
          dec     A
          sub     A,YH          ; ACC NOW HAS INDEX-PREC-1
          ror     A             ; DIVIDE BY 2, LOW ORDER BIT TO CARRY (CARRY WAS SET)
          MOV     YL,A
          ldi     YH,0xFF
          push    A
          in      r25,SREG     ; PUSH PSW
          push    r25           ; CARRY BIT SAYS WHETHER DIGIT IN LEFT OR RIGHT 
          lds     ZL,VALUE     ;LHLD
          lds     ZH,VALUE+1
          sbiw    YL,1          ; ACCOUNT FOR SIGN BYTE
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          ld      A,Z
          brcs    GFD1
          ror     A
          ror     A
          ror     A
          ror     A
;*
GFD1:     andi    A,15
          ldi     c_tmp,'0'        ; ADI
          add     A,c_tmp
          ret
;*   
;*   
;*    FREE FORM CONVERT
;*   
;*   
FREE:     push    ZL
          push    ZH
          ldi     ZL,low(FPNIB*256+WMAX)
          ldi     ZH,high(FPNIB*256+WMAX); 0x81A  SET UP CWIDTH AND CFRACT
          sts     CWIDTH,ZL
          sts     CWIDTH+1,ZH
          pop     ZH
          pop     ZL
          ld      A,Z
          or      A,A
          breq    IFRM1
          cpi     A,0x89        ; 200Q+FPNIB+1
          brcc    EFORM         ; EXPONENT TOO BIG FOR IFORM OR FFORM
          push    ZL
          push    ZH
;*   
;*  NOW WE KNOW EXPONENT IS LEE THAN 200Q+FPNIB+1
          sbiw    ZL,1          ; PASS SIGN BYTE
          mov     XH,A
;*   
FR1:      inc     XH            ; BUMP COUNT (INITIALLY EXPONENT VALUE)
          sbiw    ZL,1          ; HL TO LAST UNCHECKED BYTE OF COEFFICIENT
          ld      A,Z
          andi    A,15          ; MASK DOWN TO RIGHT DIGIT
          brne    FR2           ; JUMP IF NO MORE ZEROES
          inc     XH
          ld      A,Z
          or      A,A
          breq    FR1           ; LOOP IF LEFT DIGIT ALSO A ZERO
;*   
;*  NOW WE HAVE INCREASED EXPONENT (IN B) BY NUMBER OF
;*  TRAILING ZEROS
;*   
FR2:      pop     ZH
          pop     ZL
          mov     A,XH
          cpi     A,0x81
          brcs    EFORM         ; JUMP IF STILL WON'T FIT
          rcall   ITEST
; PC no branch -507
          breq    PC+2
          rjmp    FF            ; JUMP IF NOT INTEGER
          rjmp    IFRM1         ; ENTER IFORM
;*   
;*   
;*     I FORMAT CONVERT
;*   
;*  
IFORM:    rcall   ITEST         ; PRESERVES HL
; PC no branch 1565
          breq    PC+2
          rjmp    FOERR         ; JUMP IF WAS NOT INTEGER
;*   
IFRM1:    eor     A,A
          sts     CFRACT,A
          lds     A,CWIDTH
          inc     A             ; MAKE ROOM FOR THE DPOINT
          rcall   FFORM
          ldi     A,'"'
          sts     EOBUF,A       ; EOBUF-1  SHORTEN THE RESULT BY 1 (REMOVING THE  DECIM
          rjmp    FP1
;*   
;*   
;*   
;*   
;*     E FORMAT CONVERT
;*
;% this function was extesivly modified in the PTDos version
;*
EFORM:    ld      A,Z           ; EXPONENT OF VALUE
          sts     EXP,A         ; SAVE IT
          ldi     c_tmp,0x81
          st      Z,c_tmp       ; SET FOR SCIENTIFIC NOTATION
          lds     A,CWIDTH
          subi    A,5           ;SUI; LEAVE ROOM FOR EXPONENT
          rcall   FFORM         ; (EXPONENT MAY BE ADJUSTED HERE.)
;*
EF3:      lds     ZL,EPOINT     ;LHLD
          lds     ZH,EPOINT+1
          ld      A,Z           ; SEE IF LAST NON-BLANK WAS A '.'
          cpi     A,'.'
          breq    EF0           ; SKIP OVER INX IF IT WAS
          adiw    ZL,1
;*
EF0:      ldi     c_tmp,'E'
          st      Z,c_tmp
          adiw    ZL,1
          lds     A,EXP
          ldi     XH,'+'        ; TEST THE EXPONENT.
          or      A,A           ; TEST IT FOR 0
          breq    EF1           ; JUMP IF ZERO CASE
;% MOV A,E  GET BACK THE REAL EXPONENT.
;% MVI D,0  THE NEXT SUBTRACT WOULD HAVE MADE D 0 (IF 16-BIT SUB).
          subi    A,0x81           ;SUI; SUBTRACT OFF BIAS+1
          brpl    EF1           ; JUMP IF POSITIVE (NON-NEG) EXPONENT
;*
;*  HERE WAS NEGATIVE EXPONENT
          ldi     XH,'-'
          com     A
          inc     A             ; TWO'S COMPLEMENT
;*
EF1:      st      Z,XH          ; SIGN CHARACTER
          adiw    ZL,1
          push    ZL
          push    ZH
          MOV     YL,A          ; LOW ORDER EXPONENT VALUE
          ldi     YH,0
;*
          lds     ZL,DPOINT     ;LHLD
          lds     ZH,DPOINT+1   ; POINTER TO DECIMAL POINT IN RESULT
          sbiw    ZL,1
          sbiw    ZL,1          ; DECREMENT POINTER TWO
          ld      A,Z           ; WE ARE CHECKING FOR OVERFLOW FROM SCIENTIFIC NOT
          cpi     A,'1'
          brne    EF2           ; JUMP IF NO OVERFLOW
;*
;*  HERE HAD OVERFLOW, SO BUMP EXPONENT AND MOVE THE SIGN
          sbiw    ZL,1          ; POINTS TO SIGN (IF '-')
          ld      A,Z
          ldi     c_tmp,' '
          st      Z,c_tmp
          adiw    ZL,1
          st      Z,A           ; MOV OVER THE SIGN (OR ' ')
          adiw    ZL,1
          sts     RESTP,ZL
          sts     RESTP+1,ZH
          ldi     c_tmp,'1'
          st      Z,c_tmp       ; OVERWRITE THE 0 WITH 1
          adiw    YL,1          ; BUMP EXPONENT
;*
EF2:      _XCHG                 ; EXPONENT TO HL
          pop     YH
          pop     YL
          ldi     A,' '         ; A=' ' FOR LEADING BLANKS
          rcall   CNS3          ; ADD THREE DIGIT EXPONENT TO STRING
          ldi     A,'"'         ; STRING TERMINATOR
          st      Y,A
          rjmp    FP1
;*
;*     CONVERT TEXT TO FLOATING POINT NUMBER
;*     DE=CHARACTER POINTER TO FIRST CHARACTER
;*     HL=RESULT ADDRESS
;*     RETURNS UPDATED POINTER IN DE (PAST TERMINATOR,
;*        WITH TERMINATOR IN A)
;*   
FPIN:     eor     A,A
          sts     CSIGN,A
          sts     ESIGN,A       ; EXPONENT SIGN
          ldi     A,0x80
          sts     EXP,A         ; EXPONENT VALUE
          ldi     A,0xF7        ; -FPNIB-1
          sts     PUTFL,A       ; FLAG FOR THE PUT SUBROUTINE
;% DCX D
;% FPINQ INX D
          ld      A,Y           ; FIRST CHARACTER
;%  CPI ' '
;%  JZ FPINQ  SKIP SPACES
          cpi     A,'+'
          breq    FPIN1
          cpi     A,0xCB        ; PLSRW
          breq    FPIN1
          cpi     A,0xCC        ; MINRW
          breq    FPIN0
          cpi     A,'-'
          brne    FPIN2
;*
FPIN0:    ldi     A,1
          sts     CSIGN,A
;*
FPIN1:    adiw    YL,1          ; INCREMENT POINTER PAST THE SIGN CHARACTER
;*
FPIN2:    sts     RESTA,ZL
          sts     RESTA+1,ZH    ; ADDRESS OF RESULT
          ldi     A,4           ; FPBYT
          sbiw    ZL,1          ; ACCOUNT FOR SIGN AND EXP BYTES
;* 
FPIN3:    sbiw    ZL,1
          ldi     c_tmp,0
          st      Z,c_tmp
          dec     A
          brne    FPIN3
          sts     RESTP,ZL
          sts     RESTP+1,ZH
          _XCHG
          sts     STRPT,ZL
          sts     STRPT+1,ZH    ; STRING  POINTER ADDRESS
          rcall   GNC1
          brcs    FIS21         ; WAS DIGIT
          cpi     A,'.'
          brne    FIFAIL
;*
;*     DP CAME FIRST, MAKE SURE AT LEAST ONE DIGIT FOLLOWS
;*   
          rcall   GNC1
          brcs    FIS41
;*
FIFAIL:   sec
          ret
;*   
;*     HAVE DP, NO SD YET
;*   
FIS4:     ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          _DCR_M
          breq    FIFAIL        ; WENT TO ZERO, UNDERFLOW
;*   
FIS40:    rcall   GNC
;*   
FIS41:    breq    FIS4          ; HAVE A ZERO SO DECREMENT EXPONENT AND GET NE
; PC no branch 94
          brcc    PC+2
          rjmp    FIS60
          cpi     A,'E'
; PC no branch 100
          brne    PC+2
          rjmp    FIS7
;*   
;*     GET HERE MEANS WE MUST HAVE A ZERO RESULT
;*   
FIZERO:   eor     A,A
          sts     CSIGN,A       ; CAN'T HAVE NEG ZERO!
          rjmp    FIDN2
;*   
;*     EAT LEADING ZEROS, IF THERE ARE ANY
;*   
FIS2:     rcall   GNC
;*   
FIS21:    breq    FIS2
          brcs    FIS5
          cpi     A,'.'
          breq    FIS40
          rjmp    FIZERO
;*   
;*     HAVE SD BEFORE DP
;*   
FIS5:     rcall   PUTD
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          ld      A,Z
          ldi     c_tmp,1        ; ADI
          add     A,c_tmp       ; FOR THE CARRY
          st      Z,A
          brcs    FIFAIL        ; > 127, OVERFLOW
          rcall   GNC
          brcs    FIS5
          cpi     A,'.'
          breq    FIS6
          cpi     A,'E'
          breq    FIS7
;*   
;*     HERE WE ARE DONE, SO PUT THE NUMBER TOGETHER
;*   
FIDN:     eor     A,A
;*   
FIDN1:    ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          ld      c_tmp,Z
          add     A,c_tmp
; PC no branch -84
          brcc    PC+2
          rjmp    FIFAIL        ; OVERFLOW
; PC no branch -87
          brne    PC+2
          rjmp    FIFAIL        ; UNDERFLOW
;*   
FIDN2:    lds     ZL,RESTA     ;LHLD
          lds     ZH,RESTA+1
          clc     ;or      A,A           ; CLEAR CARRY FOR A GOOD RETURN
          st      Z,A           ; PLACE EXP
          lds     A,CSIGN
          sbiw    ZL,1
          st      Z,A           ; PLACE SIGN
          lds     ZL,STRPT     ;LHLD
          lds     ZH,STRPT+1
          _XCHG                 ; POINTER BACK TO DE
          lds     A,TERMC
          mov     XH,A
          ret                   ; CARRY IS CLEAR
;*   
;*     HERE WE HAVE A SD AND A DP
;*   
FIS6:     rcall   GNC
          brcc    FIS61
;*   
FIS60:    rcall   PUTD
          rjmp    FIS6
;*   
FIS61:    cpi     A,'E'
          brne    FIDN
;*   
;*     HERE WE HAVE NZ COEF AND 'E'
;*   
FIS7:     rcall   GNC
          brcs    FIS81
          cpi     A,'+'
          breq    FIS8
          cpi     A,0xCB        ; PLSRW
          breq    FIS8
          cpi     A,'-'
          breq    FIS71
          cpi     A,0xCC        ; MINRW
; PC no branch -149
          breq    PC+2
          rjmp    FIFAIL
;*   
FIS71:    sts     ESIGN,A       ; PUTS NON-ZERO VALUE IN ESIGN
;*   
;*     NOW GET THE EXPONENT MAGNITUDE
;*   
FIS8:     rcall   GNC           ; GET FIRST DIGIT
; PC no branch -158
          brcs    PC+2
          rjmp    FIFAIL        ; COMPLAIN IF NO FIRST DIGIT
;*   
FIS81:    MOV     YL,A          ; SAVE IN E IN CASE OF MUL10
          ldi     YH,0
;*   
FIS99:    rcall   GNC           ; SEE IF ANOTHER DIGIT
          brcc    FIS82
          rcall   MUL10
; PC no branch -173
          brcc    PC+2
          rjmp    FIFAIL        ; WAY TOO BIG!
          rjmp    FIS99         ; MUL10 MAY EXIT IF NUMERIC OVERFLOW
;*   
FIS82:    ldi     ZL,low(-127)
          ldi     ZH,high(-127) ; TEST SIZE OF EXPONENT
          add     ZL,YL     ; DAD D
          adc     ZH,YH
; PC no branch -183
          brcc    PC+2
          rjmp    FIFAIL        ; TOO BIG!
          lds     A,ESIGN       ; TEST SIGN
          or      A,A
          mov     A,YL          ; GET EXP
; PC no branch -114
          brne    PC+2
          rjmp    FIDN1         ; DONE, PACK AND LEAVE
          com     A
          inc     A             ; TWO'S COMP
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          ld      c_tmp,Z
          add     A,c_tmp
; PC no branch -200
          brcs    PC+2
          rjmp    FIFAIL        ; UNDERFLOW
; PC no branch -203
          brne    PC+2
          rjmp    FIFAIL        ; UNDERFLOW
          rjmp    FIDN2
;*   
;*     SPECIAL GET CHARACTER ROUTINE FOR FPIN
;*     EXPEXTS POINTER IN STRPT
;*     IF DIGIT, SETS CARRY AND CLEANS TO 4 BITS (Z SET IF 0)
;*   
GNC:      lds     ZL,STRPT     ;LHLD
          lds     ZH,STRPT+1
GNC1:     ld      A,Z
          sts     TERMC,A
          adiw    ZL,1
;% CPI ' '  IGNORE SPACES
;% JZ GNC1
          sts     STRPT,ZL
          sts     STRPT+1,ZH
          cpi     A,':'
          brcs    PC+2          ; RNC
          ret
          cpi     A,'0'
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcs    PC+2          ; RNC
          ret
          andi    A,15
          sec
          ret
;*   
;*     ROUTINE FOR FPIN TO PUT NEXT DIGIT IN RESULT
;*     CLEAN DIGIT IS IN ACC
;*     SCALAR PUTFL HAS COUNT OF DIGITS ALREADY PUT, SO IF TOO BI
;*     THEN DIGIT IS NOT PUT (THE PREC + 1 DIGIT IS USED TO ROUND
;*
PUTD:     mov     XH,A
          ldi     ZL,low(PUTFL)
          ldi     ZH,high(PUTFL)
          _INR_M
          ld      A,Z           ; LOAD COUNT TO ACC
          lds     ZL,RESTP     ;LHLD
          lds     ZH,RESTP+1    ; POINTER TO CURRENT BYTE IN VALUE
          breq    PUTD1         ; ROUNDING DIGIT
          brmi    PC+2          ; RP
          ret                   ; WE ARE BEYOND ROUNDING DIGIT IF POSITIVE
          lsr     A      ;RRC -- check cary rotate
          brcs    PUTR
;*   
;*     HERE WE WANT TO PUT DIGIT IN LEFT NIBBLE
;*
          mov     A,XH
		  swap A
 ;         lsl     A      ;RLC -- check cary rotate
  ;        lsl     A      ;RLC -- check cary rotate
   ;       lsl     A      ;RLC -- check cary rotate
    ;      lsl     A      ;RLC -- check cary rotate
          st      Z,A
          ret
;*   
;*     HERE PUT DIGIT INTO RIGHT NIBBLE
;*   
PUTR:     ld      A,Z
          or      A,XH
          st      Z,A
          adiw    ZL,1
          sts     RESTP,ZL
          sts     RESTP+1,ZH
          ret
;*   
;*     CAME HERE IF ROUNDING DIGIT
;*   
PUTD1:    mov     A,XH          ; ROUNDING DIGIT TO ACC
          cpi     A,5
          brcc    PC+2          ; RC
          ret
          sbiw    ZL,1
          ldi     XL,4
;*   
BUMP:     ldi     A,1
          ld      c_tmp,Z
          add     A,c_tmp
          call    DAA ;-- decimal accumulator adjust convert to BCD
          st      Z,A
          brcc    PUTD90
          sbiw    ZL,1
          dec     XL
          brne    BUMP
;*   
PUTD90:   brcs    PC+2          ; RNC
          ret                   ; RETURN IF CARRY NOT PROPOGATED WHOLE DISTANCE
;*   
;*     HERE MUST CHANGE TO 1.0000000, HL HAS MSB-1
;*   
          adiw    ZL,1
          ldi     c_tmp,16
          st      Z,c_tmp
          ldi     ZL,low(EXP)
          ldi     ZH,high(EXP)
          ld      A,Z
          ldi     c_tmp,1        ; ADI
          add     A,c_tmp
          st      Z,A
; PC no branch -294
          brcc    PC+2
          rjmp    FIFAIL        ; OVERFLOW IF > 127 (REMEMBER BIAS?)
          ret

;% end of file BSM#CVT2
cSINP:
SYSTS:
	sei
	wdr
;Check for input stored in the terminal input buffer	
	lds idx,TRMIEI				; get input empty index
	lds c_tmp,TRMIFI			; any input stored?
	cp idx,c_tmp
	breq SYSTS_99

;Get next character from terminal input buffer
;	MOV	TRMIBF(A5),A6		; set buffer index
	ldi ZL,low(TRMIBF)
	ldi ZH,high(TRMIBF)

	add ZL,idx
	adc ZH,zero

	;munge if needed
	ld A,Z
	inc idx
	andi idx,0x1F				; 32 byte test buffer
	sts TRMIEI,idx

SYSTS_99:
	ret

;*
;*
;*     THE  INPUT/OUTPUT  HANDLERS
;*
;*
;% start of file BSM#IO
;*   
;*   
;*     OUTPUT FILTER #1
;*      CHARACTER IS IN B
;*   
OF1:      lds     A,LEDFG
          or      A,A
          breq    OF13          ; TRANSLATE THIS CHARACTER? 
;*   
          eor     A,A           ; CLEAR LEAD IN FLAG
          sts     LEDFG,A
          mov     A,XH
          cpi     A,'&'         ; &&=&
;*   
          breq    CHOUT         ; OF2  JUST PRINT IT
          ldi     A,0xC0
          add     A,XH          ; TRANSLATE CHARACTER TO CTRL-CHARACTER
          mov     XH,A
          rjmp    CHOUT         ; OF2  PRINT CHARACTER
;*
OF13:     mov     A,XH
          cpi     A,'&'         ; LEAD IN CHARACTER?
          brne    CHOUT         ; NO, PRINT CHARACTER
          sts     LEDFG,A
          ret
;*   
;*   
;*     OUTPUT FILTER #2
;*      ALIAS "CHOUT"
;*       CHARACTER IS IN B
;*   
; IF SOLOS
;XSYOUT EQU $
; ENDF
;OF2 EQU $
; PC no jump -26176
CHOUT:    call    cSYSOT        ; SEND IT OUT
          mov     A,XH
          subi    A,13           ;SUI
          breq    OF21
          mov     A,XH
          cpi     A,' '
          brcc    PC+2          ; RC
          ret                   ; NO PHEAD INC FOR CTRL-CHARACTERS
          lds     A,PHEAD
          inc     A
;*
OF21:     sts     PHEAD,A
          ret
;*
;*
;*     CARRIAGE RETURN
;*     IF NEEDED
;*
CCRLF:    lds     A,PHEAD
          or      A,A
          brne    PC+2          ; RZ
          ret
;*   
;*     CARRIAGE RETURN
;*     ALWAYS
;*   
CRLF:     ldi     XH,13
          rcall   CHOUT
          ldi     XH,10
          rjmp    CHOUT
;*   
;*     CHECK IF PANIC CHARACTER HAS BEEN HIT
;*   
; IF SOLOS
;XSYT2 EQU $
; ENDF
; PC no jump -26206
PCHECK:   call    cSINP
          brne    PC+2          ; RZ
          ret
;*   
;*   
;*     INPUT FILTER #3
;*      MASKS OFF PARITY AND CHECKS FOR ESCAPE
;*       CHARACTER RETURNED IN A
;*   
;PCHK1 EQU $
;IF3 EQU $
PCHK1:    andi    A,0x7F        ; DIRTY LITTLE BIT!
          cpi     A,0           ; ESC   % the break char not to be confused with the other one
          breq    PC+2          ; RNZ
          ret
;*   
;*     CHECK TO SEE IF PANIC HAPPENED DURING RUN
;*   
          push    XL
          push    XH
          push    YL
          push    YH
          push    ZL
          push    ZH            ; INCASE OF STOP SO CCONT CAN CONTINUE
          lds     A,DIRF
          or      A,A
; PC no branch -6211
          brne    PC+3
          jmp     STOP1        ; MUST HAVE BEEN RUN
          ldi     XH,'\\'        ; MUST HAVE BEEN INPUT (OR LIST)
          rcall   CHOUT
          rcall   CRLF
; PC no jump -9865
          jmp     CMND1         ; ABORT
;*   
;*   
;*    SPEED CONTROL CHECKING
;*   
; IF SOLOS
;XSYT6 EQU $
; PC no jump -26236
SPDCK:    call    cSINP
          brne    PC+2          ; RZ
          ret
          andi    A,0x7F
          cpi     A,' '
          brne    SPD1          ; WAIT IF SPACE BAR
;XSYT3 EQU $
; PC no jump -26247
SPD0:     call    cSINP
          breq    SPD0
SPD1:     cpi     A,'1'
          brcs    PCHK1
          cpi     A,':'
          brcc    PCHK1
;*   
;*  1-9 IS SPEED CONTROL
;*   
          subi    A,'0'           ;SUI
          mov     XH,A
          eor     A,A
SPED:     sec
          rol     A
          dec     XH
          brne    SPED
          ror     A
; PC no jump -7592
          jmp     SETDS         ; SET DISPLAY SPEED
; ENDF
;*   
;*   
;*     INPUT FILTER #1
;*      CHARACTER IS IN A AND B
;* 
IF1:      lds     A,CTLFG       ; GET CTRL-CHARACTER FLAG
          or      A,A
          breq    IF12          ; NO CTRL-CHARACTER EXPANSION
          mov     XH,A
          eor     A,A
          mov     XL,A          ; ZERRO DISPATCH FLAG
          sts     CTLFG,A       ; CLEAR CTRL-CHARACTER EXPANTION FLAG
          mov     A,XH
          ret                   ; NOTE: CARRY IS CLEAR
;*
IF12:     rcall   IF2           ; GET CHARACTER
;*
          ldi     XL,8          ; SCHRZ   SPECIAL CHARACTER TABLE SIZE (IN ENTRIES)
          ldi     ZL,low(SCHRT*2)
          ldi     ZH,high(SCHRT*2); ADDR OF SPECIAL CHARACTER TABLE
SCTS:     lpm
          mov c_tmp,r0
          adiw    ZL,1          ; POINT TO ADDR OF ROUTINE
          cp      A,c_tmp       ; A=M?
          sec                   ; INCASE OF MATCH
          brne    PC+2          ; RZ
          ret                   ; A=M, CARRY<>0 MEANS DISPATCH
          adiw    ZL,1
          adiw    ZL,1
          dec     XL
          brne    SCTS          ; SPECIAL CHARACTRER TABLE SEARCH
;*
          cpi     A,' '         ; CTRL-CHARACTER?
          brcs    PC+2          ; RNC
          ret                   ; NO, RETURN CARRY IS 0
;*
          ldi     c_tmp,'@'        ; ADI
          add     A,c_tmp       ; CONVERT TO NON-CTRL-CHARACTER
          sts     CTLFG,A       ; SAVE FOR EXPANTION
          ldi     A,'&'
          mov     XH,A
          ret                   ; NOTE:  CARRY IS 0
;*   
;*   
;*     INPUT FILTER #2
;*      CHARACTER IN A AND B
;*   
IF2:      push    ZL
          push    ZH
IF20:     ldi     ZL,low(ITIMCONST)
          ldi     ZH,high(ITIMCONST)
; IF SOLOS
;XSYT4 EQU $
; ENDF
; PC no jump -26326
IF21:     call    cSINP         ; SYSTS
          brne    IF22          ; THERE IS A CHARACTER
;*
          lds     A,DIRF
          or      A,A
          brne    IF21          ; NO TIMING IN DIRECT MODE
;*
          sbiw    ZL,1          ; TICK...TOCK
          mov     A,ZH
          or      A,ZL
          brne    IF21
          lds     ZL,ITIM     ;LHLD
          lds     ZH,ITIM+1     ; GET INPUT TIME LIMIT
          mov     A,ZH
          or      A,ZL
          breq    IF20          ; NOT ACTIVE
          sbiw    ZL,1
          sts     ITIM,ZL
          sts     ITIM+1,ZH     ; UPDATE TIME LIMIT
          mov     A,ZH
          or      A,ZL
          breq    IF23          ; TIME'S UP SINKER...SINKER
          rjmp    IF20          ; NOT YET
;*
IF22:     rcall   PCHK1         ; IF3  MASK AND TEST FOR ESCAPES
          mov     XH,A
          pop     ZH
          pop     ZL
          ret
;*
IF23:     ldi     A,13          ; THIS IS WHAT YOU GET FOR TAKING SO LONG
          mov     XH,A
          pop     ZH
          pop     ZL
          ret
;% cassette basic calls here
PLF:      _XCHG
;*  
;*   
;*     INPUT A LINE FROM THE TERMINAL
;*   
;PCR EQU $
; IF SOLOS
PCR:      _XCHG                 ; TERMINATE AT END OF LINE (DE)
;ENDF
          ldi     c_tmp,13
          st      Z,c_tmp       ; TERMINATE AT PRESENT POSITION (HL)
          inc     XL            ; THERE IS ALWAYS ROOM FOR A CR!  (THIS WON'T BE CORRECT!)
;*
          mov     XH,A
          lds     A,TOPT        ; GET OLD PHEAD
          add     A,XL
          sts     PHEAD,A
;*
          lds     A,DIRF
          or      A,A
          mov     A,XH
          breq    PC+2          ; RNZ
          ret                   ; NO TIME/COUNT CLEAR IF IN DIRECT MODE
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     ITIM,ZL
          sts     ITIM+1,ZH     ; THIS ZEROS ITIM
          sts     ITIM+1,ZL
          sts     ITIM+1+1,ZH   ; THIS ZEROS ICNT
          ret                   ; RETURN C HAS LINE LENGTH
;*
INL0:     rcall   CRLF          ; ENTRY THAT RE-GETS A LINE
;*
;*  THIS IS THE ENTRY TO GET A LINE
;* 
INLINE:   lds     A,PHEAD
          sts     TOPT,A        ; SAVE PHEAD
; IF SOLOS
          rcall   READR         ; GET VDM ADDRESS TO 'VDMAD'
; ENDF
INL9:     ldi     ZL,low(IBUF1)
          ldi     ZH,high(IBUF1); READ CHARACTRERS INTO IBUF
          mov     YH,ZH         ; HL=PRESENT POSITION, DE=LAST CHARACTER POSITION
          mov     YL,ZL         ; INITIALY SAME AT FIRST
          eor     A,A
          mov     XL,A          ; C COUNTS THE NUMBER OF CHARACTERS IN THE LINE
;*
INL1:     lds     A,DIRF
          or      A,A
          brne    INL1B         ; NO COUNT LIMIT IN DIRECT MODE
;*
          lds     A,ICNT        ; GET INPUT COUNT LIMIT
          or      A,A           ; A  SET?
          breq    INL1B         ; NO, CONTINUE 
          cp      A,XL          ; EQUAL?
; PC no branch -65
          brne    PC+2
          rjmp    PCR           ; COUNT'S UP....SNIKER...SINKER
;*   
INL1B:    push    ZL
          push    ZH            ; GET NEXT CHARACTER
          push    XL
          push    XH
          push    YL
          push    YH
          rcall   IF1           ; GET CHARACTER
          pop     YH
          pop     YL
          pop     XH
          pop     XL
          mov     XH,A
          brcs    INLSD         ; SPECIAL DISPATCH
          pop     ZH
          pop     ZL
;*   
          lds     A,INSFG       ; TEST INSERT FLAG
          or      A,A
; PC no branch 108
          breq    PC+2
          rjmp    INSRT         ; INSERT ON
;*   
          _HDCMP                ; CMP HL-DE
          brne    INL2          ; DIDN'T PASS DE (POINTER NEXT EMPTY CELL)
          mov     A,XL
          cpi     A,LINMAX-2    ; 0x82
          brcc    INL1          ; GREATER THAN OR EQU TO, DONT KEEP IT
          adiw    YL,1
          inc     XL
;*   
INL2:     st      Z,XH          ; PUT IN BUFFER
          rcall   CHOUT         ; OF2  PRINT IT
          adiw    ZL,1
          rjmp    INL1          ; NEXT CHARACTER
;*  
;*   
;*     DISPATCH TO A SPECIAL CHARACTER HANDLER
;*   
INLSD:    rcall   pgm_LHLI          ; GET ROUTINE ADDRESS TO HL
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          ret
;*   
;*     PROCESS BACK SPACEING
;*   
PBS:      _HDCMP         ; AT END OF LINE?
          brne    PBS0
          eor     A,A
          cp      A,XL
; PC no branch -83
          brne    PC+2
          rjmp    INL9
          sbiw    ZL,1
          sbiw    YL,1
          dec     XL
PBS9:     ldi     XH,'_'        ; THIS GOES BACK ON THE SOLOS VDM DRIVER
          rcall   CHOUT
          rjmp    INL1
;*   
PBS0:     dec     XL            ; ONE LESS CHARACTER
; PC no branch -98
          brne    PC+2
          rjmp    INL9
          push    ZL
          push    ZH
;*   
PBS1:     adiw    ZL,1          ; SHIFT IT ALL DOWN
          ld      A,Z
          sbiw    ZL,1
          st      Z,A
          adiw    ZL,1
          _HDCMP
          brne    PBS1
          sbiw    YL,1          ; EOL ADDR
          rjmp    INST2         ; CHR POINTER
; IF SOLOS
;*   
;*     PROCESS LEFT CURSOR MOVEMENT
;*   
PLFT:     push    YL
          push    YH
          ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1)
          _HDCMP                ; ALL THE WAY LEFT ALREADY?
          pop     YH
          pop     YL
; PC no branch -118
          brne    PC+2
          rjmp    INL1          ; YES, IGNORE
          sbiw    ZL,1
; ENDF
;*
PCKEY:    ldi     A,0x80        ; TURN ON HIGH BIT FOR SOLOS
          or      A,XH
          mov     XH,A
;*
PECHO:    rcall   CHOUT         ; OF2  PRINT CHARACTER
          rjmp    INL1          ; CONTINUE
;*   
;*     PROCESS RIGHT MOVEMENT
;*   
PRIT:     _HDCMP
; PC no branch -135
          brcs    PC+2
          rjmp    INL1          ; ALREADY ALL THE WAY OVER!
          adiw    ZL,1          ; MOVE
          rjmp    PCKEY
;*
;*     PROCESS INSERT MODE ON/OFF
;*   
PIOFF:    eor     A,A           ; INSERT MODE OFF
PION:     sts     INSFG,A       ; SET INSERT FLAG
          rjmp    INL1
;*   
;*     INSERT MODE
;*   
INSRT:    mov     A,XL
          cpi     A,LINMAX-2    ; 0x82  DOES IT FIT?
; PC no branch -152
          brcs    PC+2
          rjmp    INL1          ; NO, IGNORE COMMAND
          inc     XL
          adiw    ZL,1
          adiw    YL,1
          push    XL
          push    XH            ; SAVE CHARACTER (IN B)
; IF SOLOS
          ldi     XH,KRIGHT     ; 19 (0x13)  MOVE CURSOR RIGHT ONE
; ENDF
          rcall   CHOUT         ; OF2  IF PTDOS, ECHO CHR; ELSE MOVE CURSOR
          pop     XH
          pop     XL
          push    ZL
          push    ZH
          sbiw    ZL,1
;*   
INST1:    ld      A,Z           ; SHIFT CHARACTERS UP
          st      Z,XH
          mov     XH,A
          _HDCMP                ; DONE?
          adiw    ZL,1
          brcs    INST1         ; NO, KEEP LOOPING
;*
; IF SOLOS
INST2:    lds     ZL,XOPORT     ;LHLD
          lds     ZH,XOPORT+1
          ld      A,Z           ; TEST FOR NON-VDM OUTPUT PORT
          or      A,A
          brne    INST4         ; NOT ON VDM, DON'T CHANGE SCREEN
;*  
          push    YL
          push    YH            ; SAVE POINTER TO END OF BUFFER
          push    XL
          push    XH            ; SAVE CHARACTER COUNT AND INPUT CHARACTER
          ldi     YL,low(IBUF1)
          ldi     YH,high(IBUF1); BUFFER POINTER
          lds     ZL,VDMAD     ;LHLD
          lds     ZH,VDMAD+1    ; SCREEN POINTER
          mov     A,XL          ; CHARACTER COUNT TO MOVE
          cpi     A,'@'         ; 63+1
          brcs    INST3
          ldi     XL,'?'        ; 63  MOVE A MAX OF 63 CHARACTERS
;*   
INST3:    _XCHG
          ld      XH,Z          ; CHARACTER FROM IBUF1
          _XCHG
          ld      A,Z           ; CHARACTER FROM SCREEN
          andi    A,0x80        ; EXTRACT "CURSOR" BIT
          or      A,XH          ; ADD "CURSOR" BIT TO B
          st      Z,A           ; CHARACTER TO VDM SCREN
          adiw    ZL,1
          adiw    YL,1
          dec     XL            ; DONE?
          brne    INST3
;*   
          ld      A,Z           ; FORCE LAST CHARACTER TO BLANK
          andi    A,0x80        ; PRESERVE "CURSOR" BIT
          ori     A,' '         ; MAKE LAST CHARACTER BLANK
          st      Z,A
;*   
          pop     XH
          pop     XL
          pop     YH
          pop     YL
; ENDF
INST4:    pop     ZH
          pop     ZL
          rjmp    INL1          ; DONE INSERTING/DELETING, CONTINUE
; IF SOLOS
;*   
;*   
;*     READ VDM SCREEN ADDRESS
;* 
READR:    eor     A,A
          sts     INSFG,A       ; INIT INSFG TO OFF
          ldi     XH,27         ; KESC
          rcall   ZOUT
          ldi     XH,4          ; THE PROPER CODE TO GET ADDRESS
          rcall   ZOUT
          mov     ZH,XH
          mov     ZL,XL
          sts     VDMAD,ZL
          sts     VDMAD+1,ZH    ; SAVE ADDRESS
          ret
;*   
;*  OUTPUT TO PORT ZERO (VDM)
;* 
ZOUT:     eor     A,A
;XAOUT EQU $
; PC no jump -26672
          jmp     cAOUT         ; AOUT  OUTPUT AND RETURN
;*   
; ENDF
;*
;*
;% end of BSM#IO
;*
;*
;*     THE  ERROR  PROCESSOR
;*
;*
;% start of BSM#ERR
;*
;*
;*  ERROR MESSAGE HANDLER
;*
NIERR:    ldi     XL,low(0x4E49)
          ldi     XH,high(0x4E49); NOT IMPLEMENTED
          rjmp    ERROR
UNERR:    ldi     XL,low(0x5544)
          ldi     XH,high(0x5544); REF TO UNDEFINED VAR
          rjmp    ERROR
ACERR:    ldi     XL,low(0x4143)
          ldi     XH,high(0x4143); FILE ACCESS ERROR
          rjmp    ERROR
; IF SOLOS
OPERR:    ldi     XL,low(0x4F50)
          ldi     XH,high(0x4F50); OPEN ERROR
          rjmp    ERROR
CLERR:    ldi     XL,low(0x434C)
          ldi     XH,high(0x434C); CLOSE ERROR
          rjmp    ERROR
; ENDF
CAERR:    ldi     XL,low(0x4341)
          ldi     XH,high(0x4341); CAN'T APPEND FROM A SEMI-COMPILED FILE
          rjmp    ERROR
; IF SOLOS
WTERR:    ldi     XL,low(0x5754)
          ldi     XH,high(0x5754); WRITE ERROR
          rjmp    ERROR
; ENDF
RDERR:    ldi     XL,low(0x5244)
          ldi     XH,high(0x5244); READ ERROR (DATA AND FILE)
          rjmp    ERROR
NPERR:    ldi     XL,low(0x4E50)
          ldi     XH,high(0x4E50); NO PROGRAM
          rjmp    ERROR
LLERR:    ldi     XL,low(0x4C4C)
          ldi     XH,high(0x4C4C); LINE LENGTH
          rjmp    ERROR
BAERR:    ldi     XL,low(0x4241)
          ldi     XH,high(0x4241)
          rjmp    ERROR
DMERR:    ldi     XL,low(0x444D)
          ldi     XH,high(0x444D); DIMENSION ERROR
          rjmp    ERROR
FDERR:    ldi     XL,low(0x4644)
          ldi     XH,high(0x4644); FILE, FUNCTION, FIELD DEFINITION ERROR
          rjmp    ERROR
OBERR:    ldi     XL,low(0x4F42)
          ldi     XH,high(0x4F42); ARGUMENT OUT OF BOUNDS
          rjmp    ERROR
TYERR:    ldi     XL,low(0x5459)
          ldi     XH,high(0x5459);  VAR, CONST TYPE
          rjmp    ERROR
FMERR:    ldi     XL,low(0x464D)
          ldi     XH,high(0x464D); FORMAT
          rjmp    ERROR
FOERR:    ldi     XL,low(0x464F)
          ldi     XH,high(0x464F); FIELD OVERFLOW
          rjmp    ERROR
LNERR:    ldi     XL,low(0x4C4E)
          ldi     XH,high(0x4C4E); LINE NUMBER
          rjmp    ERROR
EOFERR:   lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
          sbiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          rjmp    ERROR
DZERR:    ldi     XL,low(0x445A)
          ldi     XH,high(0x445A)
          rjmp    ERROR
BSERR:    ldi     XL,low(0x4253)
          ldi     XH,high(0x4253)
;*
;*
ERROR:    lds     A,UNDEF
          eor     A,A
          breq    ERM0
;*
;*  HERE UN-DEFINE THE MOST RECENT SYMBOL
          lds     ZL,STNBA     ;LHLD
          lds     ZH,STNBA+1
          sts     STA,ZL
          sts     STA+1,ZH
          lds     ZL,STNPTR     ;LHLD
          lds     ZH,STNPTR+1
          ldi     YL,low(0x0000)
          ldi     YH,high(0x0000)
          _DSTOR
;*
ERM0:     mov     ZH,XL
          mov     ZL,XH
          sts     LSTERR,ZL
          sts     LSTERR+1,ZH   ; SAVE LAST ERROR SEEN
          ldi     ZL,low(LSTERR)
          ldi     ZH,high(LSTERR)
          sts     IBUF,ZL
          sts     IBUF+1,ZH     ; SAVE ADDR OF MES IN IBUF
          ldi     ZL,low(SHORT)
          ldi     ZH,high(SHORT)
          ldi     c_tmp,13
          st      Z,c_tmp       ; INCASE OF A SHORT MESS
;*
          lds     A,DIRF
          or      A,A
; PC no branch 86
          breq    PC+2
          rjmp    FINE0         ; COMMAND MODE, NO LINE NUMBER
          ldi     c_tmp,' '
          st      Z,c_tmp       ; A LONG MESS
;*   
;*  FIND LINE NUMBER
;*   
ERM1:     lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1     ; 'STOP' ENTERS HERE
ERM2:     mov     XH,ZH
          mov     XL,ZL
          ld      YL,Z
          ldi     YH,0
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
          ldi     ZL,low(TXA)
          ldi     ZH,high(TXA)
          rcall   DCMP
          _XCHG
          brcs    ERM2
;*
          in r25,SREG
		  adiw    XL,1
          ld      A,X
          mov     ZL,A
          adiw    XL,1
          ld      A,X
          mov     ZH,A
		  out SREG,r25
;*
          ldi     YL,low(ASCLN)
          ldi     YH,high(ASCLN); ADDR TO PUT ASCII LN
          sts     LSTLIN,ZL
          sts     LSTLIN+1,ZH   ; SAVE LAST ERROR LINE NUMBER
          ldi     A,' '
; PC no jump -2336
          call    CLNS          ; CONVERT LN TO STRING
;*
          lds     A,DIRF        ; INCASE THIS WAS CALLED BY 'STOP' CHECK DIRF AGAIN
          or      A,A
          brne    FINE          ; MUST BE STOP, WRITE MESS
;*
          lds     ZL,ERRLN     ;LHLD
          lds     ZH,ERRLN+1    ; SEE IF ERRSET
          mov     A,ZH
          or      A,ZL
          breq    FINE1         ; PRINT ERROR AND ABORT PROG
;*
          _XCHG                 ; LN TO DE
          ldi     ZL,low(CMNDSP)
          ldi     ZH,high(CMNDSP); RESET STACK
          out     SPL,ZL        ;SPHL
          out     SPH,ZH
          sts     SPTR,ZL
          sts     SPTR+1,ZH
;*
          lds     ZL,MEMTOP     ;LHLD
          lds     ZH,MEMTOP+1   ; AND ARG STACK
          ldi     c_tmp,0
          st      Z,c_tmp
          sbiw    ZL,1
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
;*
          ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     ERRLN,ZL
          sts     ERRLN+1,ZH    ; CLEAR ERRSET
;*
          _XCHG
          push    ZL
          push    ZH            ; GARBAGE FOR GOTO2 TO POP OFF AS THE RETURN LINK
; PC no jump -7823
          jmp     GOTO2
;*
FINE1:    eor     A,A
          sts     CONTF,A
FINE0:    rcall   CCRLF
FINE:     lds     ZL,IBUF     ;LHLD
          lds     ZH,IBUF+1
          rcall   PRNTCR
; PC no jump -10991
          jmp     CMND1
;*
;*
;% end of file BSM#ERR
;*
;*
;*     THE  MISCELLANEOUS  ROUTINES
;*
;*
;% start if file BSM#MSCS
;% (SETM) is not defined in this revision
;*
;*     COPY BYTES FROM (HL) TO (DE) FOR (BC) BYTES
;*        (BC) MUST BE NONZERO ON ENTRY
;*
COPY:     ldi     XH,0
COPYX:    ld      A,Z
          st      Y,A
          adiw    ZL,1
          adiw    YL,1
          sbiw    XL,1
          mov     A,XH
          or      A,XL
          brne    COPYX
          ret
;*
;*     COMPARE DE AGAINST VALUE ADDRESSED BY HL
;*     CLOBBERS A ONLY
;*
DCMP:
	in r25,SREG     
	adiw    ZL,1
	out SREG,r25	; messy but that is the way
					; the code is strucured
DCMP1:
	; use a simpler compare

	in r25,SREG     
	ld ARGH,Z
	sbiw ZL,1
	ld ARGL,Z
	out SREG,r25	; still messy but that is the way
					; the code is strucured
	cp YL,ARGL
	cpc YH,ARGH    
;          mov     A,YH
;          ld      c_tmp,Z
;          cp      A,c_tmp
;          sbiw    ZL,1
;          breq    PC+2     ; RNZ
;          ret
;          mov     A,YL
;          ld      c_tmp,Z
;          cp      A,c_tmp
          ret
;*
;*     INDIRECT LOAD HL THRU HL
;*
LHLI:     push    A
          in      r25,SREG     ; PUSH PSW
          push    r25
          ld      A,Z
          adiw    ZL,1
          ld      ZH,Z
          mov     ZL,A
          pop     r25
          out     SREG,r25     ; POP PSW
          pop     A
          ret
;*
;*     INDIRECT LOAD HL THRU HL
;*	uses program memory
pgm_LHLI:
 	push A
    in r25,SREG
    push    r25
          lpm
		  mov DPL,r0
          adiw    ZL,1
          lpm
          mov     ZH,r0
		  mov ZL,DPL
          pop     r25
          out     SREG,r25
 	pop A
          ret
;*
;*     CLEAR DE BYTES OF MEMORY STARTING AT ADDR HL
;*
CLRM:     ldi     c_tmp,0
          st      Z+,c_tmp
;         adiw    ZL,1
          sbiw    YL,1
;          mov     A,YH
;          or      A,YL
          brne    CLRM
          ret
;*
;*   RETURN Z SET IFF THE NEXT ITEM IS A STRING ITEM
;*
STEST:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          push    ZL
          push    ZH            ; SAVE TXA
          ld      A,Z
          cpi     A,0x84        ; FNRW
          brne    STST2         ; NOT A FUNCTION
          adiw    ZL,1          ; SKIP FNRW FOR STRING TEST
          sts     TXA,ZL
          sts     TXA+1,ZH
STST2:    rcall   ALPH1         ; NEEDS TXA IN HL
          brcs    STST1         ; NOT AN ALPHA CHR
          rcall   DIG           ; DIGIT?  (RETURNS TXA IN HL)
          brcs    STST1         ; IF NO DIGIT
          ld      A,Z           ; NEXT CHR AFTER DIGIT
STST1:    pop     ZH
          pop     ZL            ; GET BACK TXA
          sts     TXA,ZL
          sts     TXA+1,ZH      ; RESTORE IT
          cpi     A,'"'
          brne    PC+2          ; RZ
          ret
          cpi     A,'$'
          brne    PC+2          ; RZ
          ret
          cpi     A,0x86        ; CHRRW
          brne    PC+2          ; RZ
          ret
          cpi     A,0x8A        ; STRRW
          brne    PC+2          ; RZ
          ret
          cpi     A,0x88        ; ERRRW
          ret
;*
;*     ROUTINES FOR FORMAT SPECIFICATION
;*     DEFAULT COPIED FROM CURRENT
;*
DFC:      lds     ZL,COPT     ;LHLD
          lds     ZH,COPT+1
          sts     DOPT,ZL
          sts     DOPT+1,ZH
          lds     ZL,CWIDTH     ;LHLD
          lds     ZH,CWIDTH+1
          sts     DWIDTH,ZL
          sts     DWIDTH+1,ZH
          ret
;*
;*     COPY CURRENT FORMAT FROM DEFAULT
;*
CFD:      lds     ZL,DOPT     ;LHLD
          lds     ZH,DOPT+1
          sts     COPT,ZL
          sts     COPT+1,ZH
          lds     ZL,DWIDTH     ;LHLD
          lds     ZH,DWIDTH+1
          sts     CWIDTH,ZL
          sts     CWIDTH+1,ZH
          ret
;*
;*     SET CURRENT FORMAT TO FREE FORMAT
;*
CFF:      ldi     ZL,low(0x2300)
          ldi     ZH,high(0x2300); '#'*256+0  SET OPTIONS FOR FREE FORMAT
          sts     COPT,ZL
          sts     COPT+1,ZH
          ret
;*
;*     DIMENSION STRING
;*     CALLED AFTER STLK HAS CREATED A SYM.TAB. ENTRY
;*     DE HAS NUMBER OF BYTES IN STRING
;*
DIMST:    ldi     ZL,low(0x0004)
          ldi     ZH,high(0x0004)
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; ADD IN STRING SIZE
          push    YL
          push    YH
          rcall   ASTAB
          pop     YH
          pop     YL
;*
;*     FALL THROUGH TO DSTOR TO SET MAX LENGTH FIELD AND RETURN
;*     THIS ROUTINE STORES THE CONTENTS OF DE AT ADDRESS IN HL, A
;*     NO REGISTERS OTHER THAN HL MODIFIED
;*
          _DSTOR
          ret
;*
;*     CREATE DIM TABLE FOR DEFAULT MATRIX OF ONE DIMENSION
;*
DIMM0:    ldi     ZL,low(0x0004)
          ldi     ZH,high(0x0004)
          rcall   ASTAB
          push    ZL
          push    ZH            ; SAVE SYM.TAB. PTR TO MAX LENGTH
          adiw    ZL,1
          adiw    ZL,1
          ldi     YL,low(10)
          ldi     YH,high(10)   ; ENTRY IN DIMENSION TABLE (DEFAULT SIZE OF 10)
          _DSTOR
          pop     ZH
          pop     ZL
;*
;*     FALL THROUGH TO DIMM
;*     CALLED AFTER STLK AND DIMENSION TABLE SET UP
;*     THE END OF DIMENSION TABLE MARKER AND REST OF MATRIX SYM.T
;*     NUMBER OF ELEMENT IN DE, SYM.TAB. PTR TO MAX IN HL
;*
DIMM:     push    ZL
          push    ZH
          ldi     XL,low(FPSIZ)
          ldi     XH,high(FPSIZ); % alternative equ FPSIZ
; PC no jump -3630
          call    IMUL
          _XCHG
          pop     ZH
          pop     ZL
          _DSTOR
          _XCHG
          adiw    ZL,1          ; ADD ROOM FOR END OF DIMENSION TABLE MARKER
          adiw    ZL,1
          rjmp    ASTAB         ; THE MARKER IS A ZERO
;*
;*   THIS SUBROUTINE EXCHANGES TOP OF STACK AND TXA, CLOBBERING
;*     NOTHING BUT SINK
;*
FTXA:     sts     SINK,ZL
          sts     SINK+1,ZH     ; USE SINK-SINK+3 FOR TEMPS
          pop     ZH
          pop     ZL            ; RETURN LINK
          sts     SINK+2,ZL
          sts     SINK+2+1,ZH
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          sts     TXA,ZL
          sts     TXA+1,ZH
          lds     ZL,SINK+2     ;LHLD
          lds     ZH,SINK+2+1   ; RETURN LINK
          push    ZL
          push    ZH
          lds     ZL,SINK     ;LHLD
          lds     ZH,SINK+1     ; ORIGINAL HL
          ret
;*
;*     JUNK ON END OF STATEMENT, TESTS IF NEXT CHAR IS EOS
;*     DOES NOT CLOBBER DE
;*     EATS CHARACTER AND LINE COUNT AFTER CR
;*     LEAVES NEW TXA IN HL
;*     SETS CARRY IF END OF FILE
;*
JOE:      lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z+		; use incrmenter
 ;         adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          cpi     A,0x97        ; EOSRW
          brne    PC+2          ; RZ
          ret
          cpi     A,13
          breq    JOE0
          ldi     ZL,low(IFTERM)
          ldi     ZH,high(IFTERM)
          ld      c_tmp,Z
          eor     A,c_tmp       ; SEE IF IT MATCHES EXPECTED IF TERMINATOR
; PC no branch -392
          breq    PC+2
          rjmp    BSERR
; PC no jump -7716
          call    REM           ; SCAN TO NEXT CR
          rjmp    JOE
;*   
JOE0:     ldi     A,13
          sts     IFTERM,A      ; THIS SAYS--ELSE IS NOT LEGAL AS A TERM
;*   
          ld      A,Z           ; GET NEXT CHR
          sec                   ; IN CASE OF EOF
          dec     A
          brne    PC+2          ; RZ
          ret                   ; IF SO LEAVE TXA POINTING TO EOF
		  in r25,SREG			; protect status flags
          adiw    ZL,3
;         adiw    ZL,1
;         adiw    ZL,1          ; SKIP OVER COUNT AND LINE NUMBER
          out SREG,r25			; % restore protected flags
;*   
JOE1:     sts     TXA,ZL
          sts     TXA+1,ZH
          clc ;or      A,A           ; CLEAR THE CARRY
          ret
;*
;*
NAME1:    rcall   ALPHA
          brcc    PC+2          ; RC
          ret
          rcall   NAME2
; PC no jump -3266
          call    VAR0
          breq    PC+2          ; RNZ
          ret
          rjmp    TYERR
;*
;*     GET NAME FROM TEXT
;*     SETS CARRY IF NAME NOT FOUND
;*     IF SUCCEEDS RETURNS NAME IN BC
;*
NAME:     rcall   ALPHA
          brcc    PC+2          ; RC
          ret
NAME2:    mov     XH,A
          ldi     XL,15
          rcall   DIG1
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcs    PC+2          ; RNC
          ret
          andi    A,15
          mov     XL,A
		  clc				;% not sure why flags are not working
          ret
;*
;*  LOOK FOR EITHER NUMERIC OR STRING NAME:C=1 NUMERIC C=0 STRING
;*
ANAME:    rcall   NAME
; PC no branch -451
          brcc    PC+2
          rjmp    BSERR
;* FALL THROUGH TO SNAME
;*
;*     BC RETURNED WITH NAME
;*     LOOKS FOR STRING NAME AT TXA
;*     BC CONTAINS NAME ON ENTRY
;*     SETS CARRY IF NOT A STRING NAME
;*
SNAME:    ldi     A,'$'
          rcall   SCANC
          brcc    PC+2          ; RC
          ret
          mov     A,XL
          ori     A,0x80
          mov     XL,A
          eor     A,A
          ret
;*
;*     GET FUNCTION NAME (FN RW TOKEN ALREADY EATEN)
;*     RETURN NAME IN BC, WITH C COPIED TO ACC
;*
FNAME:    rcall   ANAME
          mov     A,XL
          ori     A,'@'
          mov     XL,A
          ret
;*
;*     SEARCH FOR STATEMENT OF TYPE IN B OR C
;*     ACC HAS TYPE OF CURRENT STATEMENT
;*     RETURN TYPE FOUND IN ACC
;*     SET CARRY IF NOT FOUND BEFORE END OF PROGRAM
;*
LSTAT:    push    XL
          push    XH
; PC no jump -7835
          call    NEXTS
          rcall   JOE
          ld      A,Z+			;% we can use the incrementer
          pop     XH
          pop     XL
;          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
;*
          brcc    PC+2          ; RC
          ret                   ; END OF FILE FOUND
          cp      A,XH
          brne    PC+2          ; RZ
          ret
          cp      A,XL
          brne    LSTAT
          ret
;*
;*     SKIP OVER FUNCTION DEFINITION
;*     ON RETURN TXA POINTS TO LAST STATEMENT OF DEFINTION
;*
FEND:     lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1      ; SEE IF ONE LINER
FEND2:    ld      A,Z
          cpi     A,0xD4
          brne    PC+2          ; RZ
          ret
          cpi     A,13
          breq    FEND1
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          rjmp    FEND2
;*
;* MULTILINER CASE
FEND1:    ldi     XL,low(DEFRW*256+FNERW )
          ldi     XH,high(DEFRW*256+FNERW ); % DEFRW*256+FNERW
          rcall   LSTAT
          ldi     XL,low(0x4644)
          ldi     XH,high(0x4644)
; PC no branch -539
          brcc    PC+2
          rjmp    EOFERR
          cpi     A,0xA9
; PC no branch -580
          brne    PC+2
          rjmp    FDERR
          ret
;*
;*      GOBBLES NEXT TEXT CHARACTER IF ALPHABETIC
;*      SETS CARRY IF NOT
;*      NEXT CHAR IN ACC ON FAILURE
;*
ALPHA:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
ALPH1:    ld      A,Z
          cpi     A,'A'
          brcc    PC+2          ; RC
          ret
          cpi     A,'['
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcc    DIGT1
          ret
;*
;*     GOBBLES NEXT TEXT CHAR IF DIGIT
;*     SETS CARRY IF NOT
;*     NEXT CHAR IN ACC ON FAILURE
;*
DIG:      lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
DIG1:     ld      A,Z
          cpi     A,'0'
          brcc    PC+2          ; RC
          ret
          cpi     A,':'
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcc    PC+2          ; RC
          ret
;*   
DIGT1:    
          in r25,SREG
		  adiw    ZL,1
          out SREG,r25
		  sts     TXA,ZL
          sts     TXA+1,ZH
          ret
;*
;*     COPYS FPSIZ BYTES AT ADDR HL TO ADDR DE
;*
VCPY1:    ldi     ZL,low(FPONE)  		; weird bug as the starting byte
          ldi     ZH,high(FPONE)		; is in the odd location
		  								; this is fixed in the equates

pgm_VCOPY:
	ldi idx,6
	in r25,SREG

pgm_VCOP1:
	lpm
	st Y,r0
	sbiw Z,1
	sbiw Y,1
	dec idx
	brne pgm_VCOP1

	out SREG,r25
	ret

;*   
VCOPY:    ldi     XL,6
;*   
VCOP1:    ld      A,Z
          st      Y,A
          sbiw    ZL,1
          sbiw    YL,1
          dec     XL
          brne    VCOP1
          ret
;*
;*    LOAD PTR TO TO FP VALUE ON STACK TO BC AND HL
;*    PUTS FPSIZ IN DE
;*
TOPFP:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          ldi     XL,low(FPSIZ)
          ldi     XH,high(FPSIZ)
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          mov     YH,ZH
          mov     YL,ZL
          ret
;*
;*
;*     PUSH VALUE ADDRESSED BY HL ONTO ARG STACK
;*     SETS ARGF, CLEARS CARRY
;*
pgm_PSHAS:
; we need a copy of this func for dealing with
; rom constants pushed onto the stack    
          push    ZL
          push    ZH
          ldi     YL,low(0xFFFA)
          ldi     YH,high(0xFFFA)
          rcall   PSHCS			; allocate space on stack
          pop     YH
          pop     YL
          _XCHG
          eor     A,A
          inc     A
          sts     ARGF,A
          rjmp    pgm_VCOPY		; exit through Vcopy


PSHAS:    push    ZL
          push    ZH
          ldi     YL,low(0xFFFA)
          ldi     YH,high(0xFFFA)
          rcall   PSHCS
          pop     YH
          pop     YL
          _XCHG
          eor     A,A
          inc     A
          sts     ARGF,A
          rjmp    VCOPY
;*
;*
;*     POP ARG STACK
;*     HL CONTAINS ADDRESS TO PUT POPPED VALUE AT
;*
POPFP:    ldi     ZL,low(FPSINK)
          ldi     ZH,high(FPSINK); THIS ENTRY POINT POPS TOP FP TO FPSINK
;*
POPAS:    _XCHG
POPA1:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          ldi     XL,low(FPSIZ)
          ldi     XH,high(FPSIZ)
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          rjmp    VCOPY
;*
;*     PUSH FRAME ONTO CONTROL STACK
;*     TAKES MINUS AMOUNT TO SUB FROM TSTKA IN DE
;*     DOES OVERFLOW TEST AND RETURNS OLD TSTKA
;*
PSHCS:    lds     ZL,TSTKA     ;LHLD
          lds     ZH,TSTKA+1
          push    ZL
          push    ZH
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          rcall   STOV
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          pop     ZH
          pop     ZL
          ret
;*
;*     EXHANGE TXA AND RTXA
;*     CLOBERS NOTHING
;*
XTXA:     push    ZL
          push    ZH
          lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          push    ZL
          push    ZH
          lds     ZL,RTXA     ;LHLD
          lds     ZH,RTXA+1
          sts     TXA,ZL
          sts     TXA+1,ZH
          pop     ZH
          pop     ZL
          sts     RTXA,ZL
          sts     RTXA+1,ZH
          pop     ZH
          pop     ZL
          ret
;*
;*     ALLOCATE HL BYTES OF SYMBOL TABLE SPACE
;*     AND ZERO IT OUT
;*     RETURNS BEGINNING OF AREA ALLOCATED IN HL
;*
ASTAB:    _XCHG
ASTA1:    lds     ZL,STA     ;LHLD
          lds     ZH,STA+1
          push    ZL
          push    ZH
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          rcall   STOV
          sts     STA,ZL
          sts     STA+1,ZH
          pop     ZH
          pop     ZL
          push    ZL
          push    ZH
          rcall   CLRM
          pop     ZH
          pop     ZL
          ret
;*
;*     STORAGE OVERFLOW TEST
;*     TEST THAT VALUE IN HL IS BETWEEN TSTKA AND STA
;*     DOES NOT CLOBBER HL, DE, BE
;*
STOV:     push    YL
          push    YH
          _XCHG
          ldi     ZL,low(STA+1)
          ldi     ZH,high(STA+1)
          rcall   DCMP1
          brcs    SOERR
          ldi     ZL,low(TSTKA+1)
          ldi     ZH,high(TSTKA+1)
          rcall   DCMP1
          _XCHG
          pop     YH
          pop     YL
          brcc    PC+2          ; RC
          ret
;*   
SOERR:    ldi     XL,low(0x534F)
          ldi     XH,high(0x534F); 'SO'
          lds     ZL,MEMTOP     ;LHLD
          lds     ZH,MEMTOP+1
          sbiw    ZL,1
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH
          rjmp    ERROR
;*
;*     INCREMENTS TXA IF NEXT CHAR IS EQUAL TO B
;*     ELSE SYNTAX ERROR
;*     DOESN'T CLOBBER BC, DE
;*
EATL0:    rcall   GCI           ; SPRECIAL ENTRY POINT FOR EATLP
;*
EATLP:    ldi     XH,0xC8
          rjmp    EATC
;*
EATRP:    ldi     XH,')'        ; ENTRY POINT FOR EATING A RIGHT PAREN
;*
EATC:     lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z
          adiw    ZL,1
          sts     TXA,ZL
          sts     TXA+1,ZH
          cp      A,XH
          brne    PC+2          ; RZ
          ret
          rjmp    BSERR
;*
;*     GET NEXT TEXT CHAR FROM PROGRAM INTO ACC
;*
GC:       lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z
          ret
;*
;*     GET NEXT TEXT CHAR AND INCREMENT TXA
;*     DOES NOT CLOBBER DE, BC
;*     RETURN CHAR IN ACC
;*
GCI:      lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      A,Z
          IN R25,SREG
          adiw    ZL,1
		  OUT SREG,R25
          sts     TXA,ZL
          sts     TXA+1,ZH
          ret
;*
;*     SCAN FOR COMMA
;*
SCOMA:    ldi     A,','
;*
;*     SCAN NEXT CHAR
;*     IF IT EQUALS A THEN GOBBLE IT AND RETURN IT IN A WITH
;*     CARRY CLEAR (I.E. C BIT OPERATES IN REVERSE OF Z BIT)
;*     ELSE SET CARRY AND DON'T GOBBLE IT
;*     CLOBBERS HL
;*
SCANC:    lds     ZL,TXA     ;LHLD
          lds     ZH,TXA+1
          ld      c_tmp,Z
          cp      A,c_tmp
          sec                   ; SET CARRY
          breq    PC+2          ; RNZ
          ret
		  IN R25,SREG			; PROTECT STATUS
          adiw    ZL,1
		  OUT SREG,R25			; RESTORE STATUS
          sts     TXA,ZL
          sts     TXA+1,ZH      ; CLEAR CARRY
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          ret
;*
RADD:     add     ZL,YL     ; DAD D
          adc     ZH,YH
          dec     XL
          brne    RADD
          ret
;*
;*     FIND TEXT LINE WITH LEAST LINE NUMBER GREATER OR EQUAL TO
;*     RETURNS TEXT ADDRESS OF COUNT BYTE IN HL
;*     ALSO COMPUTES "NEW LINE NUMBER" FOR REN COMMAND
;*
FINDLN:   lds     ZL,BEG     ;LHLD
          lds     ZH,BEG+1
          sts     NLN,ZL
          sts     NLN+1,ZH      ; INITIAL "NEW LINE NUMBER"
          ldi     XH,0
          lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
;*   
FIND1:    ld      XL,Z
          mov     A,XL
          cpi     A,1
          sec
          brne    PC+2          ; RZ
          ret
 		  in r25,SREG	; this is a mess that
          adiw    ZL,1  ; needs AVR specific matching code
		  out SREG,r25
          rcall   DCMP
		  in r25,SREG
          sbiw    ZL,1
          out SREG,r25
		  brne    PC+2          ; RZ
          ret
          ldi     c_tmp,(1<<0)     ;CMC
          in      r25,SREG
          eor     r25,c_tmp
          out     SREG,r25
          brcs    PC+2          ; RNC
          ret
          push    ZL
          push    ZH            ; SAVE TEXT PTR
          push    YL
          push    YH
          lds     ZL,DEL     ;LHLD
          lds     ZH,DEL+1
          _XCHG
          lds     ZL,NLN     ;LHLD
          lds     ZH,NLN+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
; PC no branch -840
          brcc    PC+2
          rjmp    OBERR
          sts     NLN,ZL
          sts     NLN+1,ZH
          pop     YH
          pop     YL
          pop     ZH
          pop     ZL
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          rjmp    FIND1
;*
;*    FIND LINE    MUST BE EXACT MATCH
;*
;FNEQLN EQU $
;#if code revion
; LHLD BOFA  START AT BIGINING OF FILE
; MVI B,0
;FNEQ1 MOV C,M  BC:=LENGTH OF THIS LINE
; MOV A,C
; DCR A  TEST FOR EOF FLAG (01H)
; JZ LNERR
; INX H  PASS LENGTH BYTE
; CALL DCMP  DE-(HL)
; DCX H  POINT AT LENGTH BYTE
; RZ .  Z=1 & C#1, FOUND, RETURN
; DAD B
; JMP FNEQ1
;#endif
FNEQLN:   rcall   FINDLN
; PC no branch -831
          brcc    PC+2
          rjmp    LNERR
; PC no branch -834
          breq    PC+2
          rjmp    LNERR
          ret
;*
;*     GET LINE NUMBER ARGUMENTS
;*       FORMAT:
;*          COMMAND [FIRST][,][LAST]
;*     DEFAULTS:
;*          FIRST  BOFA
;*          LAST   EOFA
;*          IF ONLY FIRST IS GIVEN, LAST = FIRST + ONE LINE
;*
GLARG:    lds     ZL,BOFA     ;LHLD
          lds     ZH,BOFA+1
          sts     FIRST,ZL
          sts     FIRST+1,ZH    ; FIRST DEFAULT
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1
          sts     LAST,ZL
          sts     LAST+1,ZH     ; LAST DEFAULT
;*
          rcall   GC
          cpi     A,13
          brne    PC+2          ; RZ
          ret                   ; TAKE BOTH DEFAULTS
;*
          cpi     A,','
          breq    GLA0          ; TAKE DEFAULT FOR FIRST
;*
; PC no jump -3140
          call    INTGER        ; FIRST IS APPARENTLY PRESENT
; PC no branch -864
          brcc    PC+2
          rjmp    LNERR
          _XCHG
          rcall   FINDLN        ; MUST BE EXACT MATCH
; PC no branch -871
          brcc    PC+2
          rjmp    LNERR
          sts     FIRST,ZL
          sts     FIRST+1,ZH
          ld      A,Z           ; IF NO COMMA, LAST = FIRST + ONE LINE
; PC no jump -10073
          call    ADR           ; POINT TO NEXT LINE
          sts     LAST,ZL
          sts     LAST+1,ZH
;*
          rcall   GC
          cpi     A,13
          brne    PC+2          ; RZ
          ret
;*
          cpi     A,','
; PC no branch -870
          breq    PC+2
          rjmp    BSERR
          lds     ZL,EOFA     ;LHLD
          lds     ZH,EOFA+1     ; IF COMMA PRESENT, RESET LAST DEFAULT TO EOFA
          sts     LAST,ZL
          sts     LAST+1,ZH
GLA0:     rcall   GCI           ; PASS THE COMMA
          ld      A,Z           ; GET THE NEXT CHR
          cpi     A,13
          brne    PC+2          ; RZ
          ret
;*
; PC no jump -3187
          call    INTGER        ; AST IS APPARENTLY PRESENT
; PC no branch -911
          brcc    PC+2
          rjmp    LNERR
          _XCHG
          rcall   FINDLN
; PC no branch -918
          brcc    PC+2
          rjmp    LNERR
          ld      A,Z           ; POINT TO LINE'S END
; PC no jump -10117
          call    ADR
          sts     LAST,ZL
          sts     LAST+1,ZH
;*
          _XCHG                 ; TEST PARAMETERS
          lds     ZL,FIRST     ;LHLD
          lds     ZH,FIRST+1
          _HDCMP
; PC no branch -913
          brcs    PC+2
          rjmp    BSERR         ; LAST MUST BE > FRIST !!
;*
          rcall   GC
          cpi     A,13
; PC no branch -921
          breq    PC+2
          rjmp    BSERR
          ret
;*
;*     PRINT MESSAGE
;*      HL POINTS TO MESSAGE
;*     PRNTCR PRINTS UP TO A CR
;*     PRNT PRINTS TO A QUOTE
;*     PRN1 PRINTS UP TO WHAT'S IN C
;*
PRNTCR:   ldi     XL,13         ; UP TO A CR
          rjmp    PRN1
;*  
PRNT:     ldi     XL,'"'        ; UP TO A QOUTE
;*  
PRN1:     ld      A,Z
          mov     XH,A          ; NEXT CHARACTER TO OUTPUT
          cp      A,XL          ; TERMINATOR?
          brne    PC+2          ; RZ
          ret                   ; YES ALL DONE
;*  
          cpi     A,13          ; THIS ROUTINE CAN'T PRINT A CR
; PC no branch -938
          brne    PC+2
          rjmp    BSERR
;*  
          rcall   CHOUT         ; THIS WILL EXPAND CTRL-CHARACTERS
          adiw    ZL,1
          rjmp    PRN1          ; MORE?
;*
;*     PRINT MESSAGE
;*      HL POINTS TO MESSAGE
;*     PRNTCR PRINTS UP TO A CR
;*     PRNT PRINTS TO A QUOTE
;*     PRN1 PRINTS UP TO WHAT'S IN C
;*
pgm_PRNT:
	      ldi     XL,'"'        ; UP TO A QOUTE
;*  
pgm_PRN1:
	      lpm
		  mov A,r0
          mov     XH,A          ; NEXT CHARACTER TO OUTPUT
          cp      A,XL          ; TERMINATOR?
          brne    PC+2     ; RZ
          ret                   ; YES ALL DONE
;*  
          cpi     A,13          ; THIS ROUTINE CAN'T PRINT A CR
          brne    pgm_PRN_10
		  rjmp BSERR
;*  
pgm_PRN_10:
          rcall   CHOUT         ; THIS WILL EXPAND CTRL-CHARACTERS
          adiw    ZL,1
          rjmp    pgm_PRN1          ; MORE?
;*
;*
;*     COMPARE TWO STRINGS
;*       BC     HAS LHS SIZE
;*       DE     HAS RHS SIZE
;*       LHSBA  HAS LHS BASE ADDRESS
;*       RHSBA  HAS RHS BASE ADDRESS
;*
SCOMP:    lds     ZL,RHSBA     ;LHLD
          lds     ZH,RHSBA+1    ; PLACE RHSBA ON STACK
          push    ZL
          push    ZH
          lds     ZL,LHSBA     ;LHLD
          lds     ZH,LHSBA+1    ; GET LHS ADDR
;*
SCUM9:    mov     A,XH          ; TEST FOR END OF LHS
          or      A,XL
          breq    SCUM0
;*
          mov     A,YH          ; TEST FOR END OF RHS
          or      A,YL
          breq    SCUM1
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; HL:=LHSBA
          ld      A,Z
          sbiw    ZL,1
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; HL:=RHSBA
          ld      c_tmp,Z
          cp      A,c_tmp
          sbiw    ZL,1
;*
          sbiw    YL,1
          sbiw    XL,1          ; UPDATE COUNTS
          breq    SCUM9         ; NEXT!
          pop     ZH
          pop     ZL            ; FLAGS ARE SET
          ret                   ; ALL DONE
;*
;*
SCUM0:    pop     ZH
          pop     ZL            ; LHS HAS ENDED
          mov     A,YH          ; TEST RHS
          or      A,YL
          brne    PC+2          ; RZ
          ret                   ; THEY ARE EQUAL  Z=1, C=0
          ret                   ; LHS < RHS  Z=0, C=0
;*   
;*   
SCUM1:    pop     ZH
          pop     ZL            ; END OF RHS BUT NOT LHS
          eor     A,A
          inc     A
          sec                   ; Z=0, C=1  LHS > RHS
          ret
;*   
;*   
;% end of BSM#MSCS
;*
;*
;*     THE  USER'S  PROGRAM SPACE
;*
;*
;*
;*
;*     THE  EXTENDED MATH FUNCTIONS
;*
;*
;XFUNS EQU $
pgm_Vtemp:
	; copy the flash constant to a temporary SRAM location
          push YL
          push YH
          ldi     YL,low(VTEMP+FPSIZ-1)
          ldi     YH,high(VTEMP+FPSIZ-1)
          rcall   pgm_VCOPY
          ldi     ZL,low(VTEMP+FPSIZ-1)
          ldi     ZH,high(VTEMP+FPSIZ-1)
          pop YH
          pop YL
          ret
;% start of file BSM#FUN2
;*
;*
;*
;*
;*             FLOATING POINT MATH FUNCTIONS
;*
;*
;*                DE = # DE
;*
;*
;*        FLOATING POINT EXPONENTIAL
;*
;*     XC = EXP(LOGE(XA)*XB)    XC=BC, XA=DE, XB=HL
;*
FXPN:     ld      A,Y           ; TEST FOR ZERO XA
          or      A,A
          brne    FXP0          ; NOT ZERO
          sbiw    ZL,1          ; EST XB FOR NEG
          ld      A,Z
          or      A,A
; PC no branch 1080
          breq    PC+2
          rjmp    L_NEG         ; 0**(-N) = 1/(0**N) = ERROR
; PC no jump -5495
          jmp     RFALSE        ; RETURN A ZERO
;*
FXP0:     ld      A,Z           ; TEST FOR ZERO XB
          or      A,A
; PC no branch -5486
          brne    PC+3
          jmp     RTRUE         ; RETURN A 1
;*
          sts     XB,ZL
          sts     XB+1,ZH       ; SAVE ADDR OF XB
          _XCHG
          sts     XA,ZL
          sts     XA+1,ZH       ; SAVE ADDR OF XA
          mov     ZH,XH
          mov     ZL,XL
          sts     XC,ZL
          sts     XC+1,ZH       ; SAVE ADDR OF XC
          lds     ZL,XA     ;LHLD
          lds     ZH,XA+1
          _XCHG                 ; HL TO DE
          rcall   FLOG          ; XA:=LOGE(XA), WE ASSUME XA=XC (OR XA WILL BE...
;*                                    CLOBBERED)
;*
          lds     ZL,XC     ;LHLD
          lds     ZH,XC+1       ; GET ADDR XC
          mov     XH,ZH         ; ADDR OF XC INTO BC
          mov     XL,ZL
          _XCHG                 ; HL TO DE
          lds     ZL,XB     ;LHLD
          lds     ZH,XB+1       ; GET ADDR OF XB
; PC no jump -5183
          call    S_FMUL        ; XC:=LOGE(XA)*XB
;*
          lds     ZL,XC     ;LHLD
          lds     ZH,XC+1       ; GET ADDR OF XC
          _XCHG
          rcall   FEXP          ; XC:=EXP(LOGE(XA)*XB)
          ret
;*
;*
;*
;*      THIS ROUTINE FORMS SIN(X) BY USING
;*          SIN(X) = COS((PI/2)-X)
;*
;% CALL REDUCE  MAKE ARGUMENT SMALL.
FSIN:     push    YL
          push    YH            ; SAVE FOR AFTER SUBTRACTION.
;         mov     XH,YH         ; DE TO BC
          movw     XL,YL
;          _XCHG         ; DE TO HL
          ldi     YL,low((pgm_PITWO*2)+DIGI1)
          ldi     YH,high((pgm_PITWO*2)+DIGI1);   L_32D5
          movw    ZL,YL
		  rcall   pgm_Vtemp
          movw    YL,ZL 
          movw    ZL,XL
 
; PC no jump -5314
          call    FSUB          ; PI/2-X
          pop     YH
          pop     YL            ; RESTORE RESULT ADDRESS
;% JMP FCOS1  SKIP PAST THE REDUCE IN COSINE.
;*
;*   THIS ROUTINE FORMS THE COSINE OF A FLOATING POINT
;*   VALUE.
;*
;*  ENTRY PARAMETERS:
;*  D,E - ARGUMENT ADDRESS
;*  EXIT PARAMETERS
;*  RESULT IN ADDRESS POINTED TO BY D,E ON ENTRY
;*
;% FCOS CALL REDUCE  MAKE THE ARGUMENT REASONABLE.
;% FCOS1 EQU $
FCOS:     push    YL
          push    YH            ; SAVE RESULT ADDRESS
          ldi     ZL,low((pgm_TWOPI*2)+DIGI1)
          ldi     ZH,high((pgm_TWOPI*2)+DIGI1)
          rcall   pgm_Vtemp

          ldi     XL,low(EXPTM)
          ldi     XH,high(EXPTM); FTEM1+FPSIZ-1
; PC no jump -4914
          call    FDIV
          eor     A,A
          sts     EXPTM-1,A
          lds     A,EXPTM
          rcall   S_FRAC
          ldi     YL,low(EXPTM)
          ldi     YH,high(EXPTM)
          ldi     ZL,low((pgm_HALF*2)+DIGI1)
          ldi     ZH,high((pgm_HALF*2)+DIGI1)
          rcall   pgm_Vtemp

          rcall   FCOMP         ; COMPARE FRACTIONAL PART
          ldi     XL,low(EXPTM)
          ldi     XH,high(EXPTM); Y=X/2*PI
          mov     ZL,XL
          mov     ZH,XH
          
          push    ZL
          push    ZH
          ldi     ZL,low((pgm_CN14*2)+DIGI1)
          ldi     ZH,high((pgm_CN14*2)+DIGI1)
          rcall   pgm_Vtemp
          movw    YL,ZL
          pop     ZH
          pop     ZL

          brcs    FCOS20
          _XCHG
          ldi     ZL,low((pgm_CN34*2)+DIGI1)
          ldi     ZH,high((pgm_CN34*2)+DIGI1);   L_32C9
          rcall   pgm_Vtemp
;*
; PC no jump -5362
FCOS20:   call    FSUB
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM)
          mov     YH,ZH
          mov     YL,ZL
          ldi     XL,low(FTEMP+DIGI1)
          ldi     XH,high(FTEMP+DIGI1)
; PC no jump -5260
          call    S_FMUL        ; FTEMP=ARGUMENT FOR POLYA
          ldi     ZL,low((pgm_SINCO*2)+DIGI1)
          ldi     ZH,high((pgm_SINCO*2)+DIGI1); COEFFICIENT ADDRESS
          rcall   POLYA         ; EVALUATE POLYNOMICAL
          pop     XH
          pop     XL            ; RESULT ADDRESS
          ldi     YL,low(FTEM2+DIGI1)
          ldi     YH,high(FTEM2+DIGI1)
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM); FTEM1+DIGI1
; PC no jump -5276
          jmp     S_FMUL        ; FORM COSINE AND RETURN
;%
;%  insert REDUCE in revision here -- if wanting revised code
;*
;*
;*      FORM FLOATING POINT EXPONENT EXP**X.
;*
FEXP:     push    YL
          push    YH            ; SAVE FINAL ADDRESS
          ldi     XL,low(EXPTM)
          ldi     XH,high(EXPTM); FTEM1+DIGI1 (EXPTM)
          ldi     ZL,low((pgm_CNL10*2)+DIGI1)
          ldi     ZH,high((pgm_CNL10*2)+DIGI1);  L_32C3 LOG10(E)
		  rcall   pgm_Vtemp

; PC no jump -5286
          call    S_FMUL        ; X*LOG10(E)
;*  GET N - INTEGER PART OF VALUE
          ldi     ZL,low(EXPTM-1)
          ldi     ZH,high(EXPTM-1); EXPTM-1 (FTEM1+DIGIT-2)
          ld      A,Z           ; GET SIGN
          ldi     c_tmp,0
          st      Z,c_tmp       ; MAKE POSITIVE
          sts     SSIGN,A       ; SAVE SIGN
          adiw    ZL,1
          ld      A,Z           ; GET EXPONENT
          cpi     A,0x83        ; CHECK RANGE
; PC no branch -4492
          brcs    PC+3
          jmp     OVER          ; ARGUMENT OVERFLOW
          subi    A,0x81           ;SUI
          ldi     YL,0          ; SET INTEGER PART = 0
          brcs    FEXP20
          inc     A
          mov     XL,A
;* FORM 10**N - MAKE INTEGER PART THE EXPONENT
          lds     A,FTEM1
          mov     XH,A
          swap A
;          ror     A
 ;         ror     A
  ;        ror     A
   ;       ror     A
;*
FEXP10:   andi    A,15          ; THIS LOOP CONVERTS BCD TO BINARY (1,2 OR 3 DIGS)
          mov     YH,A
          mov     A,YL
          add     A,A           ; *2
          add     A,A           ; *4
          add     A,YL          ; *5
          add     A,A           ; *10
          add     A,YH          ; + NEW DIGIT
          MOV     YL,A
          mov     A,XH
          dec     XL
          brne    FEXP10
;*
          lds     A,EXPTM       ; GET EXPONENT
          rcall   S_FRAC        ; GET FRACTIONAL PART
;*
FEXP20:   mov     A,YL
          ldi     c_tmp,0x81        ; ADI
          add     A,c_tmp       ; FORM FINAL EXPONENT
          sts     SEXP,A        ; SAVE
;* GET MULTIPLE OF .1
          lds     A,EXPTM       ; GET EXPONENT
          ldi     XH,0          ; SET .1 MULTIPLE = 0
          cpi     A,0x80
          brcs    FEXP30        ; VALUE LESS THAN .1
          ldi     ZL,low(FTEM1)
          ldi     ZH,high(FTEM1)
          ld      A,Z           ; GET FIRST DIGIT
          mov     XH,A
          andi    A,15          ; 0x0F  MAKE DIGIT = 0
          st      Z,A
;*
FEXP30:   mov     A,XH
          sts     SVAL,A        ; SAVE TENTH DIGIT
          rcall   NORM          ; NORMALIZE VALUE
          ldi     XH,6          ; DIGI2
          ldi     ZL,low(FTEM1)
          ldi     ZH,high(FTEM1)
          ldi     YL,low(FTEMP)
          ldi     YH,high(FTEMP)
;*
FEXP40:   ld      A,Z+
          st      Y+,A           ; PUT INTO FTEMP
;          adiw    YL,1
 ;         adiw    ZL,1
          dec     XH
          brne    FEXP40
          ldi     ZL,low((pgm_EXPCO*2)+DIGI1)
          ldi     ZH,high((pgm_EXPCO*2)+DIGI1);   V_3380   F(X) COEFFICIENTS
          rcall   POLYA         ; FORM 10**Z
;* LOOKUP VALUE OF 10**Y, Y=.1,.2,...,.9
          lds     A,SVAL        ; GET TENTHS DIGIT
          andi    A,0xF0

          ldi     ZL,low((pgm_CNTEN*2)-1)
          ldi     ZH,high((pgm_CNTEN*2)-1);   V_3319 VALUE TABLE

          ldi     YL,low(DIGIT+2)
          ldi     YH,high(DIGIT+2); DIGIT+2
;*
FEXP50:   add     ZL,YL     ; DAD D
          adc     ZH,YH
          subi    A,16           ;SUI; 0x10
          brcc    FEXP50
;* FORM FINAL VALUE
          ldi     YL,low(FTEM2+DIGI1)
          ldi     YH,high(FTEM2+DIGI1)
          pop     XH
          pop     XL
          push    XL
          push    XH

          rcall   pgm_Vtemp

; PC no jump -5418
          call    S_FMUL        ; (10**Z)*(10**Y)
;* GET EXPONENT
          pop     XH
          pop     XL
          ldi     ZL,low(SEXP)
          ldi     ZH,high(SEXP)
          ld      A,Z
          st      X,A           ; PUT IN FINAL VALUE
          sbiw    ZL,1
          ld      A,Z           ; GET SIGN
;* CHECK SIGN
          or      A,A
          brne    PC+2          ; RZ
          ret                   ; SIGN POSITIVE
;% this will need pgm_Vcopy to temp locaton
;* SIGN NEGATIVE FORM EXP(-X) = 1/EXP(X)
          ldi     ZL,low(FPONE)
          ldi     ZH,high(FPONE); ONE+DIGI1 1.0
          rcall   pgm_Vtemp
          movw    YL,ZL
 
          mov     ZH,XH
          mov     ZL,XL
; PC no jump -5138
          jmp     FDIV          ; DO MATH AND RETURN
;%
;%
;% insert file BSM#FSQR
;*
;*
;* THIS ROUTINE FORMS THE SQUARE ROOT OF A FLOATING POINT
;* VALUE
;*
FSQR:     push    YL
          push    YH
          mov     XH,YH
          mov     XL,YL
;% PTDOS code enters here
;*
          _XCHG                 ; DE TO HL  SAVE RESULT ADDRESS
          sts     STAKA,ZL
          sts     STAKA+1,ZH    ; SAFE PLACE
          sbiw    XL,1          ; MOV E BACK TO SIGN OF ARGUMENT
          ld      A,X           ; GET IT
          or      A,A
; PC no branch 775
          breq    PC+2
          rjmp    L_NEG         ; NEGATIVE VALUE
          adiw    XL,1
          ld      A,X           ; GET EXPONENT
          or      A,A           ; CHECK FOR ZERO
		  clc					; clear it anyway
; PC no branch -5597
          brne    PC+3
          jmp     ZERE0         ; 0 ** .5 = 0
;*
          lsr     A             ; CALCULATE RESULT EXPONENT. FIRST, EXPONENT/2
          ldi     YH,0x7F
          brcc    FSQ10_1
          inc     A             ; FIX RESULT EXP
          rjmp    FSQ10
FSQ10_1:  inc     YH
;*
FSQ10:    ldi     c_tmp,0x40        ; ADI
          add     A,c_tmp       ; 0x40 FORM NEW EXPONENT BY ADDING 1/2 OF BIAS.
          sts     FTEMP,A       ; SAVE EXPONENT
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM); this code is not close to matching anything
          st      Z,YH
          ldi     YL,5
FSQ20_1:  sbiw    XL,1
          sbiw    ZL,1
          ld      A,X
          st      Z,A
          dec     YL
          brne    FSQ20_1
          mov     A,YH
          ldi     YL,low((pgm_A1*2)+DIGI1)
          ldi     YH,high((pgm_A1*2)+DIGI1)
          cpi     A,0x7F
          breq    FSQ30
          ld      A,Z
          ldi     YL,low((pgm_A2*2)+DIGI1)
          ldi     YH,high((pgm_A2*2)+DIGI1)
          cpi     A,0x25 ; make readable
          brcs    FSQ30
          ldi     YL,low((pgm_A3*2)+DIGI1)
          ldi     YH,high((pgm_A3*2)+DIGI1)
          cpi     A,0x50  ; make readable
          brcs    FSQ30
          ldi     YL,low((pgm_A4*2)+DIGI1)
          ldi     YH,high((pgm_A4*2)+DIGI1)
;*
FSQ30:    ldi     ZL,low(0x0006)
          ldi     ZH,high(0x0006); I'M NOT SURE.
          add     ZL,YL     ; DAD D
          adc     ZH,YH
;*
          push    ZL
          push    ZH
;*
;*  CALCULATE GUESS
;*
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM);      EXPTM in PTDOS
          ldi     XL,low(FTEM2+DIGI1)
          ldi     XH,high(FTEM2+DIGI1)
          push    XL
          push    XH
; PC no jump -5529
          push    ZL
          push    ZH
          movw    ZL,YL
          rcall   pgm_Vtemp
          movw    YL,ZL
          pop     ZH
          pop     ZL

          call    S_FMUL        ; A*X
          pop     ZH
          pop     ZL
          pop     YH
          pop     YL
          pop     XH
          pop     XL
          push    XL
          push    XH
; PC no jump -5754
          push    ZL
          push    ZH
          movw    ZL,YL
          rcall   pgm_Vtemp
          movw    YL,ZL
          pop     ZH
          pop     ZL

          call    FADD          ; A*X+B
          rcall   NEWTON        ; PERFORM NEWTON ITERATIONS
          rcall   NEWTON
;IF PX10+PX12+PX14+PX16  IF 10 DIGITS OR GREATER, ITERATE AGAIN.
          rcall   NEWTON
;ENDF
          pop     XH
          pop     XL
;% code revision
;LDAX B  SEE IF CALCULATION RESULTED IN CARRY OUT OF HIGH DIGIT.
;RRC .  SEE IF EXPONENT WENT FROM 80 TO 81
;%
          lds     A,FTEMP       ; GET RESULT EXPONENT
;% code revision
;ACI 0  BUMP EXPONENT IF CARRY OUT OCCURED.
;%
          st      X,A           ; PUT IN RESULT
          ret
;*
;* THIS ROUTINE PERFORMS A NEWTOM ITERATION TO FIND THE
;* SQUARE ROOT OF A VALUE
;*
NEWTON:   lds     ZL,STAKA     ;LHLD
          lds     ZH,STAKA+1    ; GET ARGUMENT ADDRESS
          push    ZL
          push    ZH            ; SAVE
          ldi     YL,low(EXPTM)
          ldi     YH,high(EXPTM); % same address as: FTEM1+DIGI1
          ldi     XL,low(FTEM2+DIGI1)
          ldi     XH,high(FTEM2+DIGI1)
          push    XL
          push    XH
; PC no jump -5267
          call    FDIV          ; X/Z
          pop     XH
          pop     XL
          pop     ZH
          pop     ZL
          push    ZL
          push    ZH
          push    XL
          push    XH
          mov     YH,XH
          mov     YL,XL
; PC no jump -5792
          call    FADD          ; Z+Z/X
          pop     ZH
          pop     ZL
          pop     XH
          pop     XL

          push    ZL
          push    ZH
          ldi     ZL,low((pgm_HALF*2)+DIGI1)
          ldi     ZH,high((pgm_HALF*2)+DIGI1); .5
          rcall   pgm_Vtemp
          movw    YL,ZL
          pop     ZH
          pop     ZL

; PC no jump -5582
          jmp     S_FMUL        ; .5*(Z+X/Z)
;*
;* THIS ROUTINE FORMS THE TANGENT OF A FLOATING POINT
;* VALUE. THE ARGUMENT IS IN RADIANS.
;*
FTANG:    push    YL
          push    YH            ; SAVE RESULT ADDRESS
          _XCHG                 ; DE TO HL
          ldi     YL,low(FTMP1+DIGIT+1)
          ldi     YH,high(FTMP1+DIGIT+1)
          rcall   VCOPY         ; ARG TO FTMP1
          ldi     YL,low(FTMP1+DIGIT+1)
          ldi     YH,high(FTMP1+DIGIT+1)
          rcall   FCOS          ; TEMP = COS(ARG)
          pop     YH
          pop     YL            ; RESULT ADDRESS
          push    YL
          push    YH
          rcall   FSIN          ; RESULT = SIN(ARG)
          pop     XH
          pop     XL
          mov     YH,XH
          mov     YL,XL
          ldi     ZL,low(FTMP1+DIGIT+1)
          ldi     ZH,high(FTMP1+DIGIT+1)
; PC no jump -5312
          jmp     FDIV          ; DIVIDE AND RETURN; RESULT=SIN(ARG)/COS(ARG)
;*
;*
;* THIS ROUTINE FORMS THE ARCTANGENT OF A FLOATING POINT
;* VALUE. THE ARGUMENT IS IN RADIANS.
;*
FARCTAN:  push    YL
          push    YH            ; SAVE ADDRESS
          sbiw    YL,1
          ld      A,Y           ; GET SIGN
          sts     SSIGN,A       ; SAVE SIGN
          eor     A,A
          st      Y,A           ; SET SIGN POSITIVE
;* GET INTERVAL OF ARGUMENT
          ldi     ZL,low((pgm_ATIN1*2)+DIGI1)
          ldi     ZH,high((pgm_ATIN1*2)+DIGI1);   L_33C4
          rcall   pgm_Vtemp

          pop     YH
          pop     YL            ; GET ARGUMENT
          push    YL
          push    YH
          ldi     XL,0          ; INTERVAL COUNT
          rcall   FCOMP         ; CHECK IF IN THIS INTERVAL
          brcs    ARCT10
          ldi     ZL,low((pgm_ATIN2*2)+DIGI1)
          ldi     ZH,high((pgm_ATIN2*2)+DIGI1);   L_33CA NEXT INTERVAL
          rcall   pgm_Vtemp

          pop     YH
          pop     YL
          push    YL
          push    YH
          inc     XL            ; INCREMENT INTERVAL COUNT
          rcall   FCOMP
          brcs    ARCT10
          ldi     ZL,low((pgm_ATIN3*2)+DIGI1)
          ldi     ZH,high((pgm_ATIN3*2)+DIGI1); NEXT INTERVAL
          rcall   pgm_Vtemp

          pop     YH
          pop     YL
          push    YL
          push    YH
          inc     XL
          rcall   FCOMP         ; CHECK IF IN INTERVAL 3
          brcs    ARCT10
          inc     XL            ; INTERVAL 4
;*
ARCT10:   mov     A,XL
          sts     SEXP,A        ; SAVE INTERVAL COUNT
;* FORM ADDRESS OF B(K)
          ldi     ZL,low((pgm_ATIN3*2)+DIGI1)
          ldi     ZH,high((pgm_ATIN3*2)+DIGI1); BKCN-1
          ldi     YL,low(DIGI2)
          ldi     YH,high(DIGI2); DIGI2
ARCT20:   add     ZL,YL     ; DAD D
          adc     ZH,YH         ; FORM ADDRESS
          dec     XL
          brpl    ARCT20
;* FORM X=(Z-B(K))/(Z*B(K)+1)
          pop     YH
          pop     YL            ; GET Z ADDRESS
          push    YL
          push    YH            ; SAVE Z ADDRESS
          push    ZL   ; z points to rom
          push    ZH            ; SAVE B(K) ADDRESS
          ldi     XL,low(FTEM2+DIGI1)
          ldi     XH,high(FTEM2+DIGI1)
          push    XL
          push    XH

          rcall   pgm_Vtemp   ; hope Z is correct

; PC no jump -5681
          call    S_FMUL        ; Z*B(K)
          pop     XH
          pop     XL
          mov     YH,XH
          mov     YL,XL
          ldi     ZL,low(FPONE)
          ldi     ZH,high(FPONE); ONE+DIGI1  1.0
          rcall   pgm_Vtemp

; PC no jump -5908
          call    FADD          ; FADD  Z*B(K)+1
          pop     ZH
          pop     ZL            ; B(K) ADDRESS
          pop     YH
          pop     YL            ; Z ADDRESS
          push    YL
          push    YH
          ldi     XL,low(EXPTM)
          ldi     XH,high(EXPTM); FTEM1+DIGI1
          push    XL
          push    XH

          rcall   pgm_Vtemp

; PC no jump -5813
          call    FSUB          ; Z-B(K)
          pop     XH
          pop     XL
          mov     YH,XH
          mov     YL,XL
          ldi     ZL,low(FTEM2+DIGI1)
          ldi     ZH,high(FTEM2+DIGI1); Z*B(K)+1

; PC no jump -5411
          call    FDIV          ; PUT X IN FTEM1
          ldi     XL,low(FTEMP+DIGI1)
          ldi     XH,high(FTEMP+DIGI1)
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM); FTEM1+DIGI1
          mov     YH,ZH
          mov     YL,ZL
; PC no jump -5720
          call    S_FMUL        ; X*X IN FTEMP
          ldi     ZL,low((pgm_ATANCO*2)+DIGI1)
          ldi     ZH,high((pgm_ATANCO*2)+DIGI1); ATAN COEFFICIENTS
          rcall   POLYA         ; EVALUATE POLYNOMINAL
          ldi     XL,low(FTEM2+DIGI1)
          ldi     XH,high(FTEM2+DIGI1)
          ldi     YL,low(EXPTM)
          ldi     YH,high(EXPTM); FTEM1+DIGI1
          mov     ZH,XH
          mov     ZL,XL
; PC no jump -5737
          call    S_FMUL        ; X*F(X)
;* FORM INDEX AND GET A(K)
          lds     A,SEXP        ; GET INTERVAL COUNT
          ldi     ZL,low((pgm_AKCN*2)-1)
          ldi     ZH,high((pgm_AKCN*2)-1)
          ldi     YL,low(0x0006)
          ldi     YH,high(0x0006); DIGI2
ARCT30:   add     ZL,YL     ; DAD D
          adc     ZH,YH         ; FORM ADDRESS
          dec     A
          brpl    ARCT30
;* FOR A(K)+F(X)
          ldi     YL,low(FTEM2+DIGI1)
          ldi     YH,high(FTEM2+DIGI1)
          mov     XH,YH
          mov     XL,YL
          
          rcall   pgm_Vtemp

; PC no jump -5977
          call    FADD          ; A(K)+F(X)
;* MAKE SURE SIGN IS RIGHT
          lds     A,SSIGN       ; GET ARGUMENT SIGN
          sts     SIGN,A        ; SET RESULT SIGN
; PC no jump -5925
          jmp     ADS2          ; STORE RESULT AND RETURN
;*
;*
;* THIS ROUTINE FORMS THE COMMON LOGARITHM OF A FLOATING POI
;* VALUE.
;*
FLOG10:   push    YL
          push    YH            ; SAVE ARGUMENT ADDRESS
          rcall   FLOG          ; FORM NATURAL LOG
          pop     XH
          pop     XL            ; GET ARGUMENT ADDRESS
          ldi     ZL,low((pgm_CN23*2)+DIGI1)
          ldi     ZH,high((pgm_CN23*2)+DIGI1);   L_32DB GET DIVISOR
          rcall   pgm_Vtemp
          mov     YH,XH
          mov     YL,XL
; PC no jump -5483
          jmp     FDIV          ; FORM COMMON LOG
;*
;*
;* THIS ROUTINE FORMS THE NATURAL LOGARITHM OF A FLOATING
;* POINT VALUE
;*
FLOG:     push    YL
          push    YH            ; SAVE ARGUMENT ADDRESS
          ld      A,Y           ; GET EXPONENT
          sts     SEXP,A        ; SAVE
          or      A,A
; PC no branch 434
          brne    PC+2
          rjmp    L_NEG         ; ARGUMENT = 0
          ldi     A,0x80
          st      Y,A           ; MAKE EXPONENT = 0
          sbiw    YL,1
          ld      A,Y           ; GET SIGN
          or      A,A
		  clc
; PC no branch 425
          breq    PC+2
          rjmp    L_NEG         ; ARGUMENT LESS THEN 0
          ldi     ZL,low(-DIGIT)
          ldi     ZH,high(-DIGIT); -DIGIT
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; POINT TO FIRST DIGITS
          ld      A,Z           ; GET DIGITS
          cpi     A,0x50 ;CHECK IF F .GE. .50
          ldi     ZL,low((pgm_LOGN1*2)+DIGI1)
          ldi     ZH,high((pgm_LOGN1*2)+DIGI1); L_3455
          brcc    FLO10
          ldi     XL,low(DIGI2+DIGI2)
          ldi     XH,high(DIGI2+DIGI2); DIGI2+DIGI2
          ldi     ZL,low((((-3)*DIGI2)+(pgm_LOGN*2)-1))
          ldi     ZH,high((((-3)*DIGI2)+(pgm_LOGN*2)-1));  L_340D
;*
FLO05:    add     ZL,XL     ; DAD B
          adc     ZH,XH         ; FORM ADDRESS OF N
          subi    A,16           ;SUI
          brcc    FLO05
;*
FLO10:    sts     SPNT,ZL
          sts     SPNT+1,ZH     ; SPNT  SAVE ADDRESS
          adiw    YL,1
          ldi     XL,low(EXPTM)
          ldi     XH,high(EXPTM); FTEM1+DIGI1 EXPTM
          
		  rcall   pgm_Vtemp
; PC no jump -5834
          call    S_FMUL        ; F*N
;* FORM LOG(F*N) = LOG(Z+A)
          ldi     ZL,low(FTEM1)
          ldi     ZH,high(FTEM1)
          ld      A,Z           ; GET FIRST DIGITS
          mov     XH,A
          andi    A,15          ; FORM Z = F-A
          st      Z,A
;* FORM A IN FTEM2
          ldi     ZL,low(FTEM2)
          ldi     ZH,high(FTEM2)
          mov     A,XH
          andi    A,0xF0
          sts     SVAL,A        ; SAVE A
          st      Z,A           ; PUT IN FTEM2
          adiw    ZL,1
          ldi     XH,4          ; DIGIT
FLO15:    ldi     c_tmp,0
          st      Z,c_tmp       ; FILL WITH ZEROS
          adiw    ZL,1			; %
          dec     XH			; % swapped for BRNE
          brne    FLO15
          ldi     c_tmp,0x80
          st      Z,c_tmp       ; SET EXPONENT = 0
;* NORMALIZE FTEM1 = Z
          rcall   NORM
;* FORM Y = Z/(2A+Z)
          ldi     ZL,low(FTEM2+DIGI1)
          ldi     ZH,high(FTEM2+DIGI1); GET A
          mov     YH,ZH
          mov     YL,ZL
          mov     XH,ZH
          mov     XL,ZL
          push    XL
          push    XH
; PC no jump -6096
          call    FADD          ; FTEM2 = A+A
          pop     XH
          pop     XL
          push    XL
          push    XH            ; SAVE FTEM2
          mov     YH,XH
          mov     YL,XL
          ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM); FTEM1+DIGI1 Z (EXPTM)
          push    ZL
          push    ZH            ; SAVE FTEM1
; PC no jump -6107
          call    FADD          ; FTEM2 = 2A+Z
          pop     XH
          pop     XL
          pop     ZH
          pop     ZL
          mov     YH,XH
          mov     YL,XL
          push    XL
          push    XH            ; SAVE FTEM1
; PC no jump -5599
          call    FDIV          ; FTEM1 = Y
          pop     YH
          pop     YL
          ldi     XL,low(FTEMP+DIGI1)
          ldi     XH,high(FTEMP+DIGI1)
          mov     ZH,YH
          mov     ZL,YL
; PC no jump -5906
          call    S_FMUL        ; FTEMP = Y*Y
 
          ldi     ZL,low((pgm_LOGCO*2)+DIGI1+1)   ; nasty messy odd block
          ldi     ZH,high((pgm_LOGCO*2)+DIGI1+1);  L_3406 LOG COEFFICIENTS
          rcall   POLYA         ; EVALUATE
          ldi     ZL,low(FTEM2+DIGI1)
          ldi     ZH,high(FTEM2+DIGI1)
          ldi     YL,low(EXPTM)
          ldi     YH,high(EXPTM); FTEM1+DIGI1
          mov     XH,ZH
          mov     XL,ZL
; PC no jump -5923
          call    S_FMUL        ; Y*F(Y)
;* FORM LOG(A)
          lds     A,SVAL        ; GET A
;% swap A
          swap A
;          lsr     A      ;RRC -- check cary rotate
 ;         lsr     A      ;RRC -- check cary rotate
  ;        lsr     A      ;RRC -- check cary rotate
   ;       lsr     A      ;RRC -- check cary rotate
          subi    A,5           ;SUI; A .GE. .5
          ldi     ZL,low((pgm_LOGEA*2)-1)
          ldi     ZH,high((pgm_LOGEA*2)-1);  L_345B LOG TABLE
          ldi     YL,low(DIGI2)
          ldi     YH,high(DIGI2); DIGI2
;*   
FLO20:    add     ZL,YL     ; DAD D
          adc     ZH,YH         ; FORM ADDRESS OF LOG(A)
          dec     A
          brpl    FLO20
;* FORM LOG(F) = LOG(A)+Y*F(Y)
          ldi     YL,low(FTEM2+DIGI1)
          ldi     YH,high(FTEM2+DIGI1)
          mov     XH,YH
          mov     XL,YL
          push    XL
          push    XH            ; SAVE FTEM2

          rcall   pgm_Vtemp
; PC no jump -6170
          call    FADD
;* FORM LOG(N) AND LOG(F)-LOG(N)
          lds     ZL,SPNT     ;LHLD
          lds     ZH,SPNT+1     ; ADDRESS OF N
          ldi     YL,low(DIGI2)
          ldi     YH,high(DIGI2); DIGI2
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; ADDRESS OF LOG(N)
          pop     XH
          pop     XL            ; ACCUMULATED SUM
          mov     YL,XL
          mov     YH,XH
          
		  rcall pgm_Vtemp

; PC no jump -6078
          call    FSUB          ; LOG(F)-LOG(N)
;*   FORM LOG(10**E) = 2.3025851*E
          lds     A,SEXP        ; GET EXPONENT
          mov     YL,A			; munge rotate save A in Y
		  mov     c_tmp,A		; and in temp
		  andi    c_tmp,0x80	; save carry
          lsl     A      ;RLC -- check cary rotate; CHECK SIGN
          rol     c_tmp				; rotate cary back into temp
		  or      A,c_tmp		; merge low bit back in
		  com     A
          andi    A,1
          sts     EXPTM-1,A;   SET SIGN BIT (FTEM1+DIGIT-2)
          breq    FLO25         ; POSITIVE EXPONENT
          ldi     A,0x80
          sub     A,YL          ; NEGATIVE EXPONENT
          rjmp    FLO30
;*
FLO25:    mov     A,YL
          subi    A,0x80           ;SUI
;*
FLO30:    mov     XH,A
          inc     XH
          ldi     A,0x99        ; BCD -1
          ldi     ZH,0xFF       ; -1
;*   CONVERT BINARY EXPONENT TO DECIMAL VALUE
FLO35:    ldi     c_tmp,1        ; ADI
          add     A,c_tmp
          call    DAA ;-- decimal accumulator adjust convert to BCD; MAKE DECIMAL
          brcc    FLO40
          inc     ZH            ; GET HUNDREDS DIGIT
FLO40:    dec     XH
          brne    FLO35
;*   FORM EXPONENT OF E AND NORMALIZE
          mov     ZL,A          ; CONTAIN VALUE
          ldi     XL,3          ; AT MOST 3 DIGITS
FLO50:    add     ZL,ZL     ; DAD H
          adc     ZH,ZH         ;  NORMALIZE
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH
          add     ZL,ZL     ; DAD H
          adc     ZH,ZH
          mov     A,ZH
          cpi     A,16          ; 10H  CHECK IF NORMALIZED
          brcc    FLO55
          dec     XL
          brne    FLO50
          ldi     XL,0x80       ; FORCE EXPONENT TO 0
;*
FLO55:    mov     A,XL
          ldi     c_tmp,0x80        ; ADI
          add     A,c_tmp       ; FORM EXPONENT
          _XCHG
          ldi     ZL,low(FTEM1)
          ldi     ZH,high(FTEM1)
          st      Z,YH          ; PUT IN FIRST DIGITS
          adiw    ZL,1
          st      Z,YL
          adiw    ZL,1
          ldi     XH,2          ; DIGIT-2
;*
FLO60:    ldi     c_tmp,0
          st      Z,c_tmp       ; ZERO REST OF VALUE
          adiw    ZL,1
          dec     XH
          brne    FLO60
          adiw    ZL,1
          st      Z,A           ; PUT IN EXPONENT
;*   FORM 2.302585*E
          ldi     YL,low((pgm_CN23*2)+DIGI1)
          ldi     YH,high((pgm_CN23*2)+DIGI1)
          mov     XH,ZH
          mov     XL,ZL
          push    XL
          push    XH
          
          ; looks backwards
          push    ZL
          push    ZH
          movw    ZL,YL
          rcall   pgm_Vtemp
          movw    YL,ZL
          pop     ZH
          pop     ZL
          
; PC no jump -6055
          call    S_FMUL        ; FTEM1 = VALUE
;*   FORM LOG(X) = 2.3..LOG(E)+LOG(F)-LOG(N)
          pop     YH
          pop     YL
          ldi     ZL,low(FTEM2+DIGI1)
          ldi     ZH,high(FTEM2+DIGI1)
          pop     XH
          pop     XL            ; GET RESULT ADDRESS
; PC no jump -6281
          jmp     FADD          ; ADD AND DONE
;*
;*
;*
;* CALCULATE POLYNOMIAL OF FORM A*X**I+A*X**(I-1)+...+A*X+A
;*
;*     ENTRY PARAMETERS:
;*
;*   H,L - COEFFICIENT LIST
;*   FTEMP - X
;*
;*     EXIT PARAMETERS:
;*
;*   FTEMP1 - VALUE OF POLYNOMIAL
;*
POLYA:    rcall   POLY
          mov     YH,XH
          mov     YL,XL

          rcall   pgm_Vtemp

; PC no jump -6289
          jmp     FADD
;*
;* CALCULATE POLYNOMIAL OF FORM A*X**I+A*X**(I-1)+...+A*X
;*
POLY:     ldi     XL,low(FTEM2+DIGI1)
          ldi     XH,high(FTEM2+DIGI1); FTEM2+DIGI1  RESULT ADDRESS
          push    ZL
          push    ZH	; this is the rom constant
; tricky part the first time Z points to rom
; this gets moved into X which is the result

          rcall   pgm_Vtemp			; copy constant ptr to SRAM
          ; Z now points to sram
          ; Y is trash on entry

POLY10:   push    XL
          push    XH
          ldi     YL,low(FTEMP+DIGI1)
          ldi     YH,high(FTEMP+DIGI1); X ADDRESS

; PC no jump -6082
          call    S_FMUL
          pop     XH
          pop     XL            ; still points to result
          pop     ZH
          pop     ZL
          ldi     YL,low(DIGI2+1)
          ldi     YH,high(DIGI2+1); 0x07 DIGI2+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH         ; GET NEXT COEFFICIENT
;          ld      A,Z
          lpm					; look ahead for end of sequence
          mov A,r0
          sbiw    ZL,1
          cpi     A,0xFF        ; END OF POLYNOMIAL
          brne    PC+2          ; RZ
          ret

          mov     YH,XH			; x points to return from add
          mov     YL,XL
          push    ZL			; still points to rom
          push    ZH
          push    XL			; resets return address
          push    XH
          
          rcall   pgm_Vtemp		; copies rom to SRAM trashes Z

; PC no jump -6318
          call    FADD          ; FORM NEXT TERM
          pop     XH			; old return address
          pop     XL
          mov     ZH,XH			; now is set for next mpx
          mov     ZL,XL
          rjmp    POLY10
;*
;*
;* THIS ROUTINE COMPARES TWO FLOATING POINT NUMBERS.
;* ENTRY PARAMETERS
;* D,E - FLOATING POINT VALUE
;* H,L - FLOATING POINT VALUE
;* EXIT PARAMETERS
;* CARRY - C=1 IMPLIES (DE) .LT. (HL)
;* CARRY - C=0 IMPLIES (DE) .GE. (HL)
;*
FCOMP:    ld      A,Y           ; GET EXPONENT
          ld      c_tmp,Z
          cp      A,c_tmp
          breq    PC+2          ; RNZ
          ret                   ; EXPONENTS NOT EQUAL
          ldi     XH,4          ; DIGIT
          in r25,SREG
          sbiw    ZL,1          ; SKIP OVER THE SIGNS
          sbiw    YL,1
          out SREG,r25
;*
FCOM10:   
          in r25, SREG
          sbiw    ZL,1
          sbiw    YL,1
          out SREG,r25
          ld      A,Y           ; GET DIGITS
          ld      c_tmp,Z
          sbc     A,c_tmp
          dec     XH
          brne    FCOM10
          ret
;*
;*
;*  THIS ROUTINE RETURNS THE FRACTIONAL PART OF A FLOATING
;*  POINT VALUE
;*
;*      ENTRY PARAMETERS
;*   FTEM1 - FLOATING POINT VALUE
;*   A - VALUE EXPONENT
;*
;*      EXIT PARAMETERS
;*   FTEM1 - FRACTIONAL PART OF VALUE
;*
S_FRAC:   subi    A,0x81           ;SUI; 129
          brcc    PC+2          ; RC
          ret                   ; ALREADY FRACTION
          ldi     XH,8          ; DIGIT+DIGIT
          cpi     A,8           ; DIGIT+DIGIT  CHECK IF COMPLETE INTEGER
          brcc    FRAC10
          inc     A
          mov     XH,A
;*
FRAC10:   ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM); GET EXPONENT
          ld      A,Z
          sub     A,XH          ; FORM NEW EXPONENT
          st      Z,A           ; RESTORE EXPONENT
          mov     A,XH
          lsl     A      ;RLC -- check cary rotate
          lsl     A      ;RLC -- check cary rotate
          mov     XH,A
          rcall   SHIFT         ; FORM FRACTIONAL PART AND FALL THROUGH TO NORM
;*
;*
;*   THIS ROUTINE IS USED TO NORMALIZE A FLOATING POINT
;*   VALUE.
;*
;*     ENTRY PARAMETERS
;*   FTEM1 - FLOATING POINT VALUE TO NORMALIZE
;*
;*     EXIT PARAMETERS
;*   FTEM1 - NORMALIZED VALUE
;*
NORM:     ldi     ZL,low(FTEM1)
          ldi     ZH,high(FTEM1); FTEM1  POINT TO VALUE
          ldi     XL,low(DIGIT)
          ldi     XH,high(DIGIT)
NORM10:   ld      A,Z           ; GET DIGITS
          or      A,A           ; ARE THEY ZERO
          clc
		  brne    NORM20
          adiw    ZL,1
          inc     XH
          inc     XH
          dec     XL
          brne    NORM10
          eor     A,A           ; GET A ZERO
          sts     EXPTM,A       ; SET VALUE TO ZERO
          ret
;*
NORM20:   cpi     A,0x10         
          brcc    NORM30
          inc     XH
NORM30:   ldi     ZL,low(EXPTM)
          ldi     ZH,high(EXPTM)
          ld      A,Z           ; GET EXPONENT
          sub     A,XH
          st      Z,A
          mov     A,XH
          lsl     A      ;RLC -- check cary rotate
          lsl     A      ;RLC -- check cary rotate
          mov     XH,A          ; SHIFT COUNT
          rjmp    SHIFT
;*
;*
;*
;*
SHIF3:    mov     XH,A
          eor     A,A           ; GET A ZERO
SHIF4:    ld      YH,Z
          st      Z,A
          mov     A,YH
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    SHIF4
;*
;*   THIS ROUTINE SHIFTS A FLOATING POINT VALUE
;*   LEFT AND IS USED FOR NORMALAZATION.
;*
;*     ENTRY PARAMETERS
;*   B - SHIFT COUNT - NUMBER OF DIGITS
;*   FTEM1 - VALUE TO SHIFT
;*
SHIFT:    ldi     XL,5          ; DIGI1
          ldi     ZL,low(FTEM1+3)
          ldi     ZH,high(FTEM1+3); START OF VALUE
          mov     A,XH
          subi    A,8           ;SUI
          brcc    SHIF3
          dec     XH
          brpl    PC+2          ; RM
          ret
          ;or      A,A
		  clc
SHIF2:    
          ld      A,Z
          rol     A             ; SHIFT
          st      Z,A
		  in r25,SREG
          sbiw    ZL,1
		  out SREG,r25
          dec     XL
          brne    SHIF2
          rjmp    SHIFT
;*
;*               ERROR EXIT
;*
L_NEG:    sec                   ; ERROR
          rjmp    RETER
;*
;*
RETER:    _INX_SP
          _INX_SP
; PC no branch -5423
          brcc    PC+3
          jmp    OVER          ; PRINT "FP ERROR" IFF C=1
          ret
;*
;*
;*
;*  FLOATING POINT CONSTANTS FOR EXTENDED FUNCTIONS
;*
;% Insert file BSM#CNT1
; IF PX8  IS THIS THE 8 DIGIT VERSION?
;*
;*
;*  THE 8-DIGIT CONSTANTS
;*
;...
;*
;*
;% end file insert BSM#CNT1
;*
;*
;% end file BSM#FUN2
;% begin file BSM#FUNS
;*
;*
;% end file BSM#FUNS
;*
;*
;XMAT  EQU  $
;% begin file BSM#MTC
;*
;*     BSM:MCT       MATRIX FUNCTIONS FOR BASIC
;*
;*     This file was obtained by disassembling an working
;*     version of DBASIC 1.1 (mod 0).  This was necessary
;*     because the files BSM:MCT1 and BSM:MCT2 were missing
;*     from the disks containing the source code which
;*     Proteus purchased from Processor Techonology in
;*     1980.  The disassembly was done by Charles L. Athey, III
;*     for Proteus.
;*
;*     The names of variables, labels and subroutines which have
;*     names other than VarX, LabX, and SubX were determined by
;*     using the names which other files used to reference those
;*     locations.
;*
;*     At this time I have made no attempt to comment what is
;*     taking place in these routines.
;*
;*     Date : January 22, 1981
;*
MADD:     eor     A,A
;*
;% add or subtract depending on mode
L_349B:   sts     MODE,A
          rcall   S_350B
;*
MAddSub:  push    XL
          push    XH            ; % looks like main entry to add/sub routines
          push    YL
          push    YH
          push    ZL
          push    ZH
          lds     A,MODE
          or      A,A
          breq    L_34B1
; PC no jump -6849
          call    FSUB
          rjmp    L_34B4
;*   
; PC no jump -6960
L_34B1:   call    FADD          ; % matrix add function
;% post process add/subtract adjust indexes
; X register pair should be pointing to result of add/sub
; Register pairs Y and Z are trashed.
; *   
L_34B4:   pop     ZH
          pop     ZL            ; % restore index pointers
          pop     YH
          pop     YL
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          _XCHG
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          _XCHG
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          pop     XH
          pop     XL
          push    ZL
          push    ZH
          lds     ZL,V_3893     ;LHLD
          lds     ZH,V_3893+1
          sbiw    ZL,1
          sts     V_3893,ZL
          sts     V_3893+1,ZH
          mov     A,ZH
          or      A,ZL
          pop     ZH
          pop     ZL
          brne    MAddSub
          ret
;*
MSUB:     ldi     A,1
          rjmp    L_349B
;*
S_34D5:   push    ZL
          push    ZH
          push    YL
          push    YH
          ldi     YL,low(Var3)
          ldi     YH,high(Var3)
          rcall   S_34FC
          rcall   S_34FC
          pop     ZH
          pop     ZL
          push    ZL
          push    ZH
          ldi     YL,low(Var4)
          ldi     YH,high(Var4)
          rcall   S_34FC
          rcall   S_34FC
          lds     ZL,Var3     ;LHLD
          lds     ZH,Var3+1
          mov     YL,ZL
          mov     XL,ZH
          ldi     YH,0
          mov     XH,YH
; PC no jump -5992
          call    IMUL
          sts     V_3893,ZL
          sts     V_3893+1,ZH
          pop     YH
          pop     YL
          pop     ZH
          pop     ZL
          ret
;*
S_34FC:   ld      A,Z
          or      A,A
; PC no branch 898
          brne    PC+2
          rjmp    MDERR
          st      Y,A
          adiw    ZL,1
          adiw    YL,1
          ld      A,Z
          or      A,A
; PC no branch 890
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ret
;*
S_350B:   push    XL
          push    XH
          rcall   S_34D5
;*
          ldi     XH,6          ; probably not FPSIZ
L_3511:   ld      A,Y           ; % part of matrix math package S_350B
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 877
          breq    PC+2
          rjmp    MDERR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          ld      c_tmp,Z
          cp      A,c_tmp
          adiw    ZL,1
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          adiw    ZL,1
          adiw    YL,1
          dec     XH
          brne    L_3511
;*
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); DIGI1 or FPSIZ-1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          _XCHG
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH
          pop     XH
          pop     XL
          ret
          ld      A,Z
          or      A,A
; PC no branch 851
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ld      A,Z
          or      A,A
; PC no branch 845
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ret
;*
MMUL:     push    XL
          push    XH
          rcall   S_34D5
;*
          ldi     XL,low(0x000B)
          ldi     XH,high(0x000B); 11 not 15
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          _XCHG
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
          lds     ZL,Var3     ;LHLD
          lds     ZH,Var3+1
          _XCHG
          lds     ZL,Var4     ;LHLD
          lds     ZH,Var4+1
          mov     A,ZL
          sub     A,YH
; PC no branch 815
          breq    PC+2
          rjmp    MDERR
          mov     ZL,YL
          pop     YH
          pop     YL
          rcall   S_3689
;*
L_3559:   rcall   S_386C        ; % part of matrix math package
;*
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
          rcall   S_3817
          sbiw    ZL,1
          sts     TEMP1,ZL
          sts     TEMP1+1,ZH
;*
L_3566:   rcall   S_3855        ; % part of matrix math package MMUL
          push    ZL
          push    ZH
          rcall   S_383E
          pop     YH
          pop     YL
          ldi     XL,low(TEMP2+DIGI1)
          ldi     XH,high(TEMP2+DIGI1)
; PC no jump -6934
          call    S_FMUL
          ldi     YL,low(TEMP2+DIGI1)
          ldi     YH,high(TEMP2+DIGI1)
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1
          mov     XH,ZH
          mov     XL,ZL
; PC no jump -7163
          call    FADD
          ldi     ZL,low(V_38B1)
          ldi     ZH,high(V_38B1)
          _INR_M
          ldi     ZL,low(V_38AF)
          ldi     ZH,high(V_38AF)
          _INR_M
          lds     A,Var10
          ld      c_tmp,Z
          cp      A,c_tmp
          brne    L_3566
          eor     A,A
          st      Z,A
          sts     V_38B1,A
          ldi     ZL,low(V_38B7)
          ldi     ZH,high(V_38B7)
          _INR_M
          ldi     ZL,low(V_38B3)
          ldi     ZH,high(V_38B3)
          _INR_M
          lds     A,Var4+1
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch -73
          breq    PC+2
          rjmp    L_3559
          eor     A,A
          st      Z,A
          sts     V_38B7,A
          ldi     ZL,low(V_38B5)
          ldi     ZH,high(V_38B5)
          _INR_M
          ldi     ZL,low(V_38AD)
          ldi     ZH,high(V_38AD)
          _INR_M
          lds     A,Var3
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch -93
          breq    PC+2
          rjmp    L_3559
          ret
;*
MINV:     push    XL
          push    XH
          ldi     YL,low(Var3)
          ldi     YH,high(Var3)
          rcall   S_34FC
          rcall   S_34FC
          adiw    ZL,1
          adiw    ZL,1
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
          lds     ZL,Var3     ;LHLD
          lds     ZH,Var3+1
          sts     Var4,ZL
          sts     Var4+1,ZH
          mov     A,ZL
          sub     A,ZH
; PC no branch 687
          breq    PC+2
          rjmp    MDERR
          pop     YH
          pop     YL
          rcall   S_3689
          ldi     YH,0
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
; PC no jump -6226
          call    IMUL
          sts     TEMP1,ZL
          sts     TEMP1+1,ZH
          rcall   MZERO
          lds     A,Var3
          sts     TEMP2,A
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1
          _XCHG
          lds     ZL,V_38AB     ;LHLD
          lds     ZH,V_38AB+1
;*
L_35F3:   rcall   S_3825        ; % part of matrix INV package
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          lds     A,TEMP2
          dec     A
          sts     TEMP2,A
          brne    L_35F3
;*
L_3601:   lds     ZL,V_38AF     ;LHLD
          lds     ZH,V_38AF+1   ; % part of matrix math package MINV
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          sts     V_38AD,ZL
          sts     V_38AD+1,ZH
          adiw    ZL,1
          sts     V_38B1,ZL
          sts     V_38B1+1,ZH
          lds     A,Var3
          cp      A,ZL
          breq    L_362B
          eor     A,A
          sts     MODE,A
L_3619:   rcall   S_36B6        ; % part of INV matrix math package
          _INR_M
          lds     A,Var3
          ld      c_tmp,Z
          cp      A,c_tmp
          brne    L_3619
          ldi     ZL,low(V_38AF)
          ldi     ZH,high(V_38AF)
          _INR_M
          rjmp    L_3601
;*
L_362B:   lds     ZL,V_38AF     ;LHLD
          lds     ZH,V_38AF+1   ; % used by matrix math package INV
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          sts     V_38AD,ZL
          sts     V_38AD+1,ZH
          sbiw    ZL,1
          sts     V_38B1,ZL
          sts     V_38B1+1,ZH
          mov     A,ZH
          or      A,A
          brne    L_3650
          ldi     A,1
          sts     MODE,A
L_3642:   rcall   S_36B6        ; % part of matrix math package MINV
          _DCR_M
          brpl    L_3642
          ldi     ZL,low(V_38AF)
          ldi     ZH,high(V_38AF)
          _DCR_M
          rjmp    L_362B
;*   
L_3650:   rcall   S_36AC        ; % part of matrix math package
          ldi     ZL,low(TEMP1)
          ldi     ZH,high(TEMP1)
          rcall   S_3825
;*
L_3659:   rcall   S_3855        ; % part of matrix math package
;*
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 or DIGI1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          ldi     XL,low(TEMP3+DIGI1)
          ldi     XH,high(TEMP3+DIGI1)
          ldi     YL,low(TEMP1+DIGI1)
          ldi     YH,high(TEMP1+DIGI1)
; PC no jump -6881
          call    FDIV
          ldi     A,1
          sts     V_38B9,A
          ldi     ZL,low(TEMP3+DIGI1)
          ldi     ZH,high(TEMP3+DIGI1)
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          rcall   S_37A6
          lds     ZL,V_38B1     ;LHLD
          lds     ZH,V_38B1+1
          adiw    ZL,1
          sts     V_38B1,ZL
          sts     V_38B1+1,ZH
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          lds     A,Var3
          cp      A,ZL
          brne    L_3659
          ret
;*
;*
;*
S_3689:   _XCHG
          ld      A,Z
          cp      A,YL
; PC no branch 500
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          eor     A,A
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 494
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ld      A,Z
          cp      A,YH
          eor     A,A
          adiw    ZL,1
          ld      c_tmp,Z
          or      A,c_tmp
; PC no branch 485
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ld      c_tmp,Z
          or      A,c_tmp
; PC no branch 480
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          ld      c_tmp,Z
          or      A,c_tmp
; PC no branch 475
          breq    PC+2
          rjmp    MDERR
          adiw    ZL,1
          sts     V_38AB,ZL
          sts     V_38AB+1,ZH
;*
S_36AC:   ldi     ZL,low(V_38AD)
          ldi     ZH,high(V_38AD);  % part of matrix math package
          ldi     XL,low(0x000C)
          ldi     XH,high(0x000C)
          rcall   S_3817
          ret
;*
;*
S_36B6:   rcall   S_383E
          push    ZL
          push    ZH
          rcall   S_377D
; PC no branch 85
          breq    PC+2
          rjmp    L_3715
          lds     A,MODE
          or      A,A
; PC no branch 450
          breq    PC+2
          rjmp    MSERR
;*
L_36C7:   rcall   S_3855        ; % part of matrix math package S_36B6
          rcall   S_377D
          brne    L_36DE
          ldi     ZL,low(V_38B1)
          ldi     ZH,high(V_38B1)
          _INR_M
          lds     A,Var3
          ld      c_tmp,Z
          cp      A,c_tmp
; PC no branch 430
          brne    PC+2
          rjmp    MSERR
          rjmp    L_36C7
;*   
L_36DE:   ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000); % used by matrix math package
          sts     V_38AF,ZL
          sts     V_38AF+1,ZH
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          ldi     YL,low(0x0006)
          ldi     YH,high(0x0006); FPSIZ
          lds     A,Var3
          mov     XL,A
          ldi     XH,0
; PC no jump -6501
          call    IMUL
          sts     V_3893,ZL
          sts     V_3893+1,ZH
          push    ZL
          push    ZH
          rcall   S_3788
          pop     ZH
          pop     ZL
          sts     V_3893,ZL
          sts     V_3893+1,ZH
          lds     ZL,V_38A7     ;LHLD
          lds     ZH,V_38A7+1
          push    ZL
          push    ZH
          lds     ZL,V_38AB     ;LHLD
          lds     ZH,V_38AB+1
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
          rcall   S_3788
          pop     ZH
          pop     ZL
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
;*   
L_3715:   rcall   S_3855        ; % part of matrix math package
          sts     TEMP1,ZL
          sts     TEMP1+1,ZH
          push    ZL
          push    ZH
          ldi     YL,low(MODE)
          ldi     YH,high(MODE)
;*
          ldi     XH,6          ; FPSIZ
L_3721:   ld      A,Z           ; % part of matrix math package
          st      Y,A
          adiw    ZL,1
          adiw    YL,1
          dec     XH
          brne    L_3721
          pop     ZH
          pop     ZL
          rcall   S_377D
          pop     ZH
          pop     ZL
; PC no branch 72
          brne    PC+2
          rjmp    L_3779
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 or DIGI1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          ldi     A,1
          sts     V_38B9,A
          rcall   S_37A6
          ldi     ZL,low(TEMP3+DIGI1)
          ldi     ZH,high(TEMP3+DIGI1)
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          eor     A,A
          sts     V_38B9,A
          rcall   S_37A6
          rcall   S_37EA
          lds     ZL,V_38A7     ;LHLD
          lds     ZH,V_38A7+1
          push    ZL
          push    ZH
          lds     ZL,V_38AB     ;LHLD
          lds     ZH,V_38AB+1
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
          rcall   S_37EA
          pop     ZH
          pop     ZL
          sts     V_38A7,ZL
          sts     V_38A7+1,ZH
          sts     V_38A9,ZL
          sts     V_38A9+1,ZH
          lds     ZL,V_38AD     ;LHLD
          lds     ZH,V_38AD+1
          sts     V_38AF,ZL
          sts     V_38AF+1,ZH
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
          rcall   S_3817
L_3779:   ldi     ZL,low(V_38B1)
          ldi     ZH,high(V_38B1); % part of matrix math package S_36B6
          ret
;*
S_377D:   ldi     XH,6          ; FPSIZ
          eor     A,A
L_3780:   ld      c_tmp,Z
          cp      A,c_tmp       ; % part of matrix math package S_377D
          breq    PC+2          ; RNZ
          ret
          adiw    ZL,1
          dec     XH
          brne    L_3780
          ret
;*
S_3788:   rcall   S_383E        ; % part of matrix math package
          push    ZL
          push    ZH
          rcall   S_3855
          pop     YH
          pop     YL
;*
L_3790:   ld      XH,Z          ; % part of matrix math package
          ld      A,Y
          st      Z,A
          mov     A,XH
          st      Y,A
          adiw    YL,1
          adiw    ZL,1
          push    ZL
          push    ZH
          lds     ZL,V_3893     ;LHLD
          lds     ZH,V_3893+1
          sbiw    ZL,1
          sts     V_3893,ZL
          sts     V_3893+1,ZH
          mov     A,ZL
          or      A,ZH
          pop     ZH
          pop     ZL
          brne    L_3790
          ret
;*
S_37A6:   ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
          sts     V_38B7,ZL
          sts     V_38B7+1,ZH
          lds     ZL,V_38B1     ;LHLD
          lds     ZH,V_38B1+1
          sts     V_38B5,ZL
          sts     V_38B5+1,ZH
L_37B5:   rcall   S_3855        ; % part of matrix math package
          rcall   S_37D1
          rcall   S_386C
          rcall   S_37D1
          ldi     ZL,low(V_38B3)
          ldi     ZH,high(V_38B3)
          _INR_M
          ldi     ZL,low(V_38B7)
          ldi     ZH,high(V_38B7)
          _INR_M
          lds     A,Var4
          ld      c_tmp,Z
          cp      A,c_tmp
          brne    L_37B5
          ret
;*
S_37D1:   ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 or DIGI1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          mov     XH,ZH
          mov     XL,ZL
          _XCHG
          lds     ZL,TEMP2     ;LHLD
          lds     ZH,TEMP2+1
          lds     A,V_38B9
          or      A,A
          breq    L_37E6
; PC no jump -7559
          call    S_FMUL
          ret
;*   
; PC no jump -7265
L_37E6:   call    FDIV          ; % matrix math package divide
          ret
;*
S_37EA:   ldi     ZL,low(0x0000)
          ldi     ZH,high(0x0000)
          sts     V_38AF,ZL
          sts     V_38AF+1,ZH
          sts     V_38B3,ZL
          sts     V_38B3+1,ZH
L_37F3:   rcall   S_383E        ; % part of matrix math package S_37EA
          push    ZL
          push    ZH
          rcall   S_3855
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 or DIGI1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          _XCHG
          pop     ZH
          pop     ZL
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          mov     XH,YH
          mov     XL,YL
          _XCHG
; PC no jump -7706
          call    FSUB
          ldi     ZL,low(V_38AF)
          ldi     ZH,high(V_38AF)
          _INR_M
          ldi     ZL,low(V_38B3)
          ldi     ZH,high(V_38B3)
          _INR_M
          lds     A,Var4
          ld      c_tmp,Z
          cp      A,c_tmp
          brne    L_37F3
          ret
;*
S_3817:   eor     A,A
          push    YL
          push    YH
          mov     YH,A
L_381A:   mov     A,YH          ; % part of matrix math package S_3817
          st      Z,A
          adiw    ZL,1
          sbiw    XL,1
          mov     A,XL
          or      A,XH
          brne    L_381A
          pop     YH
          pop     YL
          ret
;*
S_3825:   ldi     c_tmp,16
          st      Z,c_tmp       ; 0x10
          adiw    ZL,1
          ldi     XL,low(0x0004)
          ldi     XH,high(0x0004); DIGIT
          rcall   S_3817
          ldi     c_tmp,0x81
          st      Z,c_tmp
          adiw    ZL,1
          ret
;*
S_3832:   _XCHG
          mov     XL,A
          ldi     XH,0
; PC no jump -6827
          call    IMUL
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
          _XCHG
          ret
;*
S_383E:   lds     ZL,V_38AD     ;LHLD
          lds     ZH,V_38AD+1
          lds     A,Var10
          rcall   S_3832
          lds     ZL,V_38AF     ;LHLD
          lds     ZH,V_38AF+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
; PC no jump -6849
          call    IMUL
          _XCHG
          lds     ZL,V_38A7     ;LHLD
          lds     ZH,V_38A7+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          ret
;*
S_3855:   lds     ZL,V_38B1     ;LHLD
          lds     ZH,V_38B1+1
          lds     A,Var4+1
          rcall   S_3832
          lds     ZL,V_38B3     ;LHLD
          lds     ZH,V_38B3+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
; PC no jump -6872
          call    IMUL
          _XCHG
          lds     ZL,V_38A9     ;LHLD
          lds     ZH,V_38A9+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          ret
;*
S_386C:   lds     ZL,V_38B5     ;LHLD
          lds     ZH,V_38B5+1
          lds     A,Var4+1
          rcall   S_3832
          lds     ZL,V_38B7     ;LHLD
          lds     ZH,V_38B7+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          _XCHG
; PC no jump -6895
          call    IMUL
          _XCHG
          lds     ZL,V_38AB     ;LHLD
          lds     ZH,V_38AB+1
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          ret
;*
MDERR:    ldi     XL,low(0x4D44)
          ldi     XH,high(0x4D44)
; PC no jump -3610
          jmp     ERROR
;*
MSERR:    ldi     XL,low(0x4D53)
          ldi     XH,high(0x4D53)
; PC no jump -3616
          jmp     ERROR
;*
;%CLA III dumped thes as VARXX
;In keeping with the other code these probably
;are a system of dynamic equates based on FPSIZ
;Var3:     .byte 1             ; used in MUL & MIN arg to sub
;Var10:    .byte 1             ; used in MUL
;Var4:     .byte 2             ; used in mul/min & sbrs
;V_3893:   .byte 2             ; used in add sub and inv
;TEMP1:    .byte 6             ; FPSIZ
;TEMP2:    .byte 6             ; FPSIZ
;MODE:     .byte 6             ; FPSIZ
;.equ      TEMP3   = MODE
;V_38A7:   .byte 2
;V_38A9:   .byte 2
;V_38AB:   .byte 2
;V_38AD:   .byte 2
;V_38AF:   .byte 2
;V_38B1:   .byte 2
;V_38B3:   .byte 2
;V_38B5:   .byte 2
;V_38B7:   .byte 2
;V_38B9:   .byte 1
;% end of file BSM#MTC
;% start of file BSM#MTC3
;*
;*
;*    MATRIX EXPRESSION EVALUATOR
;*
;*
;*     Some statements in this file have been changed
;*     so that the code produced will be exactly the same
;*     as that in the released and running version.  This
;*     was done for Proteus by Charles L. Athey, III (CLA III)
;*     See the comments at the start of BSM:MCT.
;*
SMATER:   ldi     ZL,low(MR)
          ldi     ZH,high(MR)
          rcall   MVAR
;*
          ldi     XH,0xD4       ; EQRW  SYNTAX
; PC no jump -2964
          call    EATC
;*
          eor     A,A           ; SET MODE...
          sts     MODE,A        ; ...TO 'NON-SCALOR'
;*
; PC no jump -2958
          call    GC            ; GET NEXT BYTE
          or      A,A           ; IS IT A TOKEN?
; PC no branch 290
          brpl    PC+2
          rjmp    SMATF         ; YES, IT IS
;*
          ldi     ZL,low(MA)
          ldi     ZH,high(MA)   ; ADDR OF 'A' MAT STATIC BLOCK
          rcall   MVAR
;*
; PC no jump -2971
          call    GC            ; TEST FOR CR OR EOS
          cpi     A,13
; PC no branch 172
          brne    PC+2
          rjmp    MCOPY         ; COPY MATRIX
          cpi     A,0x97        ; EOSRW
; PC no branch 167
          brne    PC+2
          rjmp    MCOPY         ; COPY MATRIX
;*
; PC no jump -2979
          call    GCI
          cpi     A,0xCA        ; ASKRW ; CHECK FOR LEGAL TOKEN
; PC no branch -3711
          brcc    PC+3
          jmp    BSERR
          cpi     A,0xCE        ; SLARW+1
; PC no branch -3716
          brcs    PC+3
          jmp    BSERR
          push    A
          in      r25,SREG      ; PUSH PSW
          push    r25           ; SAVE FOR LATER
;*
          ldi     A,0xC8        ; LPARRW ; TEST FOR SCALAR
; PC no jump -2984
          call    SCANC
          brcs    SMATV         ; NOT SCALAR, MUST BE VAR
; PC no jump -8891
          call    EXPRB         ; EVALUATE EXPRESSION
; PC no jump -3024
          call    EATRP
; PC no jump -3161
          call    TOPFP         ; HL:= ADDR OF RESULT
          sts     MB,ZL
          sts     MB+1,ZH       ; ADDR OF RESULT
          sts     TSTKA,ZL
          sts     TSTKA+1,ZH    ; POP!
          ldi     A,1
          sts     MODE,A        ; MODE := 'SCALAR' MODE
          rjmp    SMAT1
;*
SMATV:    ldi     ZL,low(MB)
          ldi     ZH,high(MB)   ; ADDR OF 'B' MAT STATIC BLOCK
          rcall   MVAR
;*
SMAT1:    pop     r25
          out     SREG,r25     ; POP PSW
          pop     A             ; GET OP CODE
          mov     XH,A
          lds     A,MODE
          or      A,A           ; TEST MODE FOR SCALAR
          mov     A,XH
          brne    SMATS         ; IT IS SCALAR MODE, BYE...BYE
;*
;% revised code -- match PTDOS binary
;% LXI H,MEXIT  SETUP RETURN FOR MAT MATH PACKAGE
;% PUSH H
;*	
          lds     ZL,MR     ;LHLD
          lds     ZH,MR+1       ; SET UP ARGS FOR MAT MATH PACKAGE
          adiw    ZL,1
          adiw    ZL,1
          mov     XH,ZH
          mov     XL,ZL         ; BC
;*
          lds     ZL,MB     ;LHLD
          lds     ZH,MB+1
          adiw    ZL,1
          adiw    ZL,1
          _XCHG                 ; DE
;*
          lds     ZL,MA     ;LHLD
          lds     ZH,MA+1
          adiw    ZL,1
          adiw    ZL,1          ; HL
;*
          cpi     A,0xCA        ; ASKRW ; DISPATCH ON OPERATOR
; PC no branch -1023
          brne    PC+2
          rjmp    MMUL
          cpi     A,0xCB        ; PLSRW
; PC no branch -1186
          brne    PC+2
          rjmp    MADD
          cpi     A,0xCC        ; MINRW
; PC no branch -1137
          brne    PC+2
          rjmp    MSUB
; PC no jump -3800
          jmp     BSERR         ; NO MAT DIVIDE
;*
;*   THIS CODE HANDLES SCALAR OPERATIONS
;*
SMATS:    sts     MODE,A        ; OPPERATOR BECOMES MODE
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 ; FIRST TIME
;*
SMSL:     ldi     ZL,low(SMSR)
          ldi     ZH,high(SMSR) ; PLACE RETURN ON STACK L_3977
          push    ZL
          push    ZH
          lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MADB,ZL
          sts     MADB+1,ZH     ; NEXT ELEMENT IN A
          _XCHG                 ; SETUP ARGS (MADB TO DE)
          lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MRDB,ZL
          sts     MRDB+1,ZH     ; NEXT ELE IN RESULT
          mov     XH,ZH
          mov     XL,ZL
          lds     ZL,MB     ;LHLD
          lds     ZH,MB+1       ; ARGS NOW: BC = DE $ HL (HL:= ADDR OF SCALOR)
;*
          lds     A,MODE        ; GET THE OPPERATOR
          cpi     A,0xCA        ; ASKRW
; PC no branch -7948
          brne    PC+3
          jmp     S_FMUL
          cpi     A,0xCB        ; PLSRW
; PC no branch -8171
          brne    PC+3
          jmp     FADD
          cpi     A,0xCC        ; MINRW
; PC no branch -8071
          brne    PC+3
          jmp     FSUB
; PC no jump -7663
          jmp     FDIV
;%
;% this addr gets pushed on stack for return
;*
SMSR:     lds     ZL,MRBZ     ;LHLD
          lds     ZH,MRBZ+1     ; MRBZ  R'S SIZE IN BYTES
          ldi     XL,low(0xFFFA)
          ldi     XH,high(0xFFFA); -FPSIZ
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; ONE LESS ELEMENT
          sts     MRBZ,ZL
          sts     MRBZ+1,ZH
          mov     A,ZH
          or      A,ZL          ; DONE?
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
          breq    PC+2
		  rjmp    SMSL          ; NO, REITERATE
;% revised code replaces return
;% JMP MEXIT  ALL DONE
          ret                   ; ALL DONE
;*
;*    MATRIX COPY
;*       WILL COPY ANY SIZE MAT TO ANY SIZE MAT
;*
MCOPY:    lds     ZL,MRCOL     ;LHLD
          lds     ZH,MRCOL+1    ; FIND DISPLACEMENT TO NEXT ROW IN RESULT MAT
          _XCHG
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
; PC no jump -7174
          call    IMUL          ; HL:=DE*BC FIND COL SIZE IN BYTES
          sts     XS,ZL
          sts     XS+1,ZH       ; ROW SIZE
;*
          lds     ZL,MACOL     ;LHLD
          lds     ZH,MACOL+1    ; FIND DISPLACEMENT TO NEXT ROW IN SOURCE MAT
          _XCHG
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
; PC no jump -7187
          call    IMUL
          sts     XA,ZL
          sts     XA+1,ZH       ; ROW SIZE
;*
          _XCHG
          lds     ZL,XS     ;LHLD
          lds     ZH,XS+1
; PC no jump -3719
          _HDCMP
          brcs    MUC1
          _XCHG
MUC1:     sts     XC,ZL
          sts     XC+1,ZH       ; SMALLEST NUMBER OF BYTES PER ROW
;*
          lds     ZL,MRROW     ;LHLD
          lds     ZH,MRROW+1    ; FIND SMALLEST NUMBER OF ROWS
          _XCHG
          lds     ZL,MAROW     ;LHLD
          lds     ZH,MAROW+1
; PC no jump -3736
          _HDCMP
          brcc    MUC4
          _XCHG
MUC4:     push    YL
          push    YH
          rjmp    MUC3          ; DE:=SMALLEST NUMBER OF ROWS
;*
MUC2:     push    YL
          push    YH            ; NUMBER OF TIMES TO COPY (# OF ROWS)
          lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1     ; MOVE TO NEXT ROW IN BOTH MATS
          _XCHG
          lds     ZL,XA     ;LHLD
          lds     ZH,XA+1       ; DISPLACEMENT TO NEXT ROW
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     MADB,ZL
          sts     MADB+1,ZH
;*
          lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1
          _XCHG
          lds     ZL,XS     ;LHLD
          lds     ZH,XS+1       ; DISP TO NEXT ROW
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     MRDB,ZL
          sts     MRDB+1,ZH
;*
MUC3:     lds     ZL,XC     ;LHLD
          lds     ZH,XC+1       ; LOAD ARGS FOR COPY
          mov     XH,ZH
          mov     XL,ZL
          lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1
          _XCHG
          lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1
; PC no jump -3793
          call    COPYX         ; COPY A ROW
          pop     YH
          pop     YL            ; # OF ROWS
          sbiw    YL,1
          mov     A,YH
          or      A,YL          ; ANY MORE ROWS TO COPY?
          brne    MUC2
;% revised code replaces return
;% JMP MEXIT  ALL DONE
          ret                   ; ALL DONE
;*
;*   PROCESS A MATRIX FUNCTION
;*
; PC no jump -3250
SMATF:    call    GCI
          cpi     A,0x8B        ; ZERO MATRIX?
          brne    SMATC         ; NO
;*
;*     The next 2 lines were added to change this file back
;*     to the way it was in previous versions.  It looks as
;*     though these changed would probably work.  CLA III
;*
;% CALL MZERO
;% JMP MEXIT
;*
;*
;*
MZERO:    lds     ZL,MRBZ     ;LHLD
          lds     ZH,MRBZ+1     ; RESULT'S SIZE IN BYTES
          _XCHG                 ; TO DE
          lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1     ; DATA BASE ADDRESS
; PC no jump -3787
          jmp     CLRM          ; CLEAR MEMORY
;*
;*     Added for compatability by CLA III
;*
;% RET .
;*JMP MEXIT  ALL DONE       Removed by CLA III for compatability
;*
;*
SMATC:    cpi     A,0x8C        ; CONRW ; A '1' MATRIX?
          brne    SMATD
;*
;*
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1 ; FIRST TIME
;*
MATC0:    lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; NEXT ELE
          sts     MRDB,ZL
          sts     MRDB+1,ZH
          _XCHG                 ; SETUP FOR VCOPY
; PC no jump -3452
          call    VCPY1
          lds     ZL,MRBZ     ;LHLD
          lds     ZH,MRBZ+1     ; R'S SIZE IN BYTES
          ldi     XL,low(0xFFFA)
          ldi     XH,high(0xFFFA); -FPSIZ
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; LESS ONE ELE
          sts     MRBZ,ZL
          sts     MRBZ+1,ZH
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
          mov     A,ZH
          or      A,ZL
          brne    MATC0         ; MORE...
;% revised code replaces return
;% JMP MEXIT  ALL DONE
          ret                   ; ALL DONE
;*
;*
SMATD:    cpi     A,0x8D        ; IDNRW ; MAT IDENTITY
          breq    PC+2
		  rjmp    SMATI         ; NO
;*
;*
          lds     ZL,MRCOL     ;LHLD
          lds     ZH,MRCOL+1
          _XCHG
          lds     ZL,MRROW     ;LHLD
          lds     ZH,MRROW+1
; PC no jump -3861
          _HDCMP
; PC no branch -441
          breq    PC+2
          rjmp    MDERR         ; MUST BE SQUARE
;*
          rcall   MZERO         ; FIRST ZERO IT OUT
;*
          lds     ZL,MRCOL     ;LHLD
          lds     ZH,MRCOL+1    ; NEXT PLACE ONES ON THE MAJOR DIAGONAL
          _XCHG                 ; TO FIND THE BYTES PER ROW
          adiw    YL,1          ; ADD ONE FOR OFFEST INTO ROW
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ ; ELEMENT SIZE
; PC no jump -7356
          call    IMUL
          push    ZL
          push    ZH            ; HL:= DISPLACEMENT TO NEXT ROW/COL
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1  FIRST TIME
          rjmp    MATD1
;*
MATD0:    push    XL
          push    XH            ; SAVE DISPLACEMENT
MATD1:    lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1     ; MOVE TO NEXT ROW/COL
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MRDB,ZL
          sts     MRDB+1,ZH
          _XCHG                 ; FOR VCOPY
; PC no jump -3522
          call    VCPY1
          pop     XH
          pop     XL            ; GET DISPLACEMENT
          lds     ZL,MRROW     ;LHLD
          lds     ZH,MRROW+1    ; TEST IF ALL ROWS DONE
          sbiw    ZL,1
          sts     MRROW,ZL
          sts     MRROW+1,ZH
          mov     A,ZH
          or      A,ZL
          brne    MATD0         ; MORE
;% revised code replaces return
;% JMP MEXIT  ALL DONE
          ret                   ; ALL DONE
;*
;*
SMATI:    cpi     A,0x8E
          brne    SMATT         ; NO, TRY TRN
;*
;*
; PC no jump -3401
          call    EATLP         ; GET ARGUMENT
          ldi     ZL,low(MA)
          ldi     ZH,high(MA)
          rcall   MVAR
; PC no jump -3405
          call    EATRP
;*
          lds     ZL,MABZ     ;LHLD
          lds     ZH,MABZ+1     ; MAKE A TEMP MAT
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006)
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          mov     XH,ZH
          mov     XL,ZL
          lds     ZL,STA     ;LHLD
          lds     ZH,STA+1      ; SYM TAB ADDRESS
          push    ZL
          push    ZH            ; SAVE IT FOR RESTORING
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; ADD MAT SIZE
; PC no jump -3463
          call    STOV          ; WILL IT FIT?
;*
          pop     YH
          pop     YL            ; ADDR OF DEST
          push    YL
          push    YH
          lds     ZL,MA     ;LHLD
          lds     ZH,MA+1
          adiw    ZL,1
          adiw    ZL,1          ; ADDR OS SOURCE, BC HAS LENGTH
; PC no jump -3966
          call    COPYX
;*
          lds     ZL,MR     ;LHLD
          lds     ZH,MR+1       ; SETUP TO CALL MINV
          adiw    ZL,1
          adiw    ZL,1
          mov     XH,ZH
          mov     XL,ZL
          pop     ZH
          pop     ZL            ; BC=RESULT, HL=SOURCE
          rcall   MINV          ; FIND INVERSE L_35B7
;% revised code replaces return
;% JMP MEXIT  ALL DONE
          ret                   ; ALL DONE
;*
;*
SMATT:    cpi     A,0x8F        ; TRNRW  TRN?
; PC no branch -4156
          breq    PC+3
          jmp    BSERR         ; THEN WHAT????
;*
;*
; PC no jump -3457
          call    EATLP
          ldi     ZL,low(MA)
          ldi     ZH,high(MA)
          rcall   MVAR
; PC no jump -3461
          call    EATRP
;*
          lds     ZL,MRROW     ;LHLD
          lds     ZH,MRROW+1    ; TEST MATS FOR SHAPE CONFORMABILITY
          _XCHG
          lds     ZL,MACOL     ;LHLD
          lds     ZH,MACOL+1
; PC no jump -3994
          _HDCMP
; PC no branch -574
          breq    PC+2
          rjmp    MDERR
;*
          lds     ZL,MRCOL     ;LHLD
          lds     ZH,MRCOL+1
          _XCHG
          lds     ZL,MAROW     ;LHLD
          lds     ZH,MAROW+1
; PC no jump -4007
          _HDCMP
; PC no branch -587
          breq    PC+2
          rjmp    MDERR
;*
;COMPUTE ROW SIZE IN BYTES:
          lds     ZL,MRCOL     ;LHLD
          lds     ZH,MRCOL+1
          _XCHG
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZ
; PC no jump -7498
          call    IMUL
          sts     TEMP1,ZL
          sts     TEMP1+1,ZH    ; INCREMENT TO NEXT ROW/COL IN RESULT
;*
          lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1     ; MOVE TO INITIAL POSITION IN MATS
          ldi     XL,low(0x0005)
          ldi     XH,high(0x0005); FPSIZ-1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MADB,ZL
          sts     MADB+1,ZH
          lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MRDB,ZL
          sts     MRDB+1,ZH
          sts     MODE,ZL
          sts     MODE+1,ZH     ; TEMP3  ROOT FOR RESULT'S COLUMNS
;*
          lds     ZL,MACOL     ;LHLD
          lds     ZH,MACOL+1    ; SET NUMBER OF ELEMENTS TO COPY PER COLUMN
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          rjmp    MATT0         ; INITIAL ENTRY
;*
MATT1:    lds     ZL,MRDB     ;LHLD
          lds     ZH,MRDB+1     ; MOVE TO NEXT COLUMN/ROW
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
          add     ZL,XL     ; DAD B
          adc     ZH,XH         ; NEXT COLUMN IN RESULT
          sts     MRDB,ZL
          sts     MRDB+1,ZH
          sts     MODE,ZL
          sts     MODE+1,ZH     ; TEMP3  ROOT OF COLUMN
;*
          lds     ZL,MACOL     ;LHLD
          lds     ZH,MACOL+1    ; NUMBER OF ELEMENTS PER COLUMN
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          rjmp    MATT2         ; INITIAL ENTRY FOR EACH COLUMN
;*
MATT3:    lds     ZL,MODE     ;LHLD
          lds     ZH,MODE+1     ; TEMP3  MOVE TO NEXT ELEMENT
          _XCHG
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1    ; DISP TO NEXT ELEMENT OF THIS COLUMN
          add     ZL,YL     ; DAD D
          adc     ZH,YH
          sts     MODE,ZL
          sts     MODE+1,ZH     ; NEW ROOT
;*
MATT2:    lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1     ; MOVE TO NEXT ELEMENT IN SOURCE
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
          add     ZL,XL     ; DAD B
          adc     ZH,XH
          sts     MADB,ZL
          sts     MADB+1,ZH
;*
MATT0:    lds     ZL,MODE     ;LHLD
          lds     ZH,MODE+1     ; TEMP3  LOAD DEST
          _XCHG
          lds     ZL,MADB     ;LHLD
          lds     ZH,MADB+1     ; AND SOURCE
; PC no jump -3727
          call    VCOPY         ; MOVE ELEMENT
;*
          lds     ZL,TEMP2     ;LHLD
          lds     ZH,TEMP2+1    ; NUMBER OF ELEMENTS PER COL
          sbiw    ZL,1
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          mov     A,ZH
          or      A,ZL
          brne    MATT3         ; NEX ELE
;*
          lds     ZL,MAROW     ;LHLD
          lds     ZH,MAROW+1    ; NUMBER OF ROWS
          sbiw    ZL,1
          sts     MAROW,ZL
          sts     MAROW+1,ZH
          mov     A,ZH
          or      A,ZL
; PC no branch -77
          breq    PC+2
          rjmp    MATT1         ; NEX ROW/COL
;* FALL THRU, ALL DONE
;*
;*   EXIT FROM THE MATRIX EXPRESSION EVALUATOR
;*
;%MEXIT LDA MENT
;% CMA .  CLEAR FLAG (THIS WAY TO CATCH BUGS!)
;% STA MENT  UPDATE TO SHOW WE'R OUT
MEXIT:    ret
;*
;*
;*
;*   LOOK UP AND SETUP FOR A MATRIX VARIABLE
;*      HL --> STATIC AREA
;*   
MVAR:     push    ZL
          push    ZH            ; SAVE STATIC POINTER
; PC no jump -3889
          call    NAME
; PC no branch -4321
          brcc    PC+3
          jmp    BSERR
; PC no jump -3873
          call    SNAME
; PC no branch -4327
          brcs    PC+3
          jmp    BSERR
;*
          ldi     A,' '         ; MTYPE ; LOOKUP AS MATRIX TYPE VAR
          or      A,XL
          mov     XL,A
; PC no jump -6896
          call    STLK
; PC no branch -4455
          brcc    PC+3
          jmp    UNERR         ; UNDEFINED!
;*   
;*  TEST FOR TWO DIMENSIONS
;*   
          push    ZL
          push    ZH            ; SAVE VAR POINTER
          adiw    ZL,1          ; PASS SIZE WORD
          adiw    ZL,1
          adiw    ZL,1          ; PASS FIRST DIMENTION (IT CAN'T BE ZERO)
          adiw    ZL,1
; PC no jump -4037
          _DLOAD
          mov     A,YH
          or      A,YL
; PC no branch -743
          brne    PC+2
          rjmp    MDERR         ; ONE DIMENSIONAL
; PC no jump -4045
          _DLOAD
          mov     A,YH
          or      A,YL
; PC no branch -751
          breq    PC+2
          rjmp    MDERR         ; MORE THAN TWO
          pop     ZH
          pop     ZL
;*   
;*  TEST FOR RE-DIMENSION
;*   
          sts     TEMP1,ZL
          sts     TEMP1+1,ZH
          ldi     A,0xC8        ; LPARRW
; PC no jump -3629
          call    SCANC
          brcc    MVAR2         ; REDIMENSION
;*   
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1    ; USE OLD SIZE
; PC no jump -4068
          _DLOAD         ; GET SIZE IN BYTES
          _XCHG
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          rjmp    MVAR0
;*   
;*  DO RE-DIM
;*   
; PC no jump -6754
MVAR2:    call    PFIXE         ; GET NEW ROW DIMENSION
; PC no branch -4435
          brne    PC+3
          jmp    OBERR
          push    YL
          push    YH            ; ONCE FOR TEST
          push    YL
          push    YH            ; AND AGAIN FOR STORING
          ldi     XH,','
; PC no jump -3687
          call    EATC
;*   
; PC no jump -6767
          call    PFIXE         ; GET NEW COL DIM
; PC no branch -4448
          brne    PC+3
          jmp    OBERR
          pop     ZH
          pop     ZL            ; GET ROW
          push    YL
          push    YH            ; SAVE COL
          mov     XH,ZH         ; ROW TO BC
          mov     XL,ZL
; PC no jump -7703
          call    IMUL          ; FIND NUMBER OF BYTES NEEDED
          _XCHG
          ldi     XL,low(0x0006)
          ldi     XH,high(0x0006); FPSIZE
; PC no jump -7710
          call    IMUL
;*   
          sts     TEMP2,ZL
          sts     TEMP2+1,ZH
          _XCHG                 ; RESULT IN HL TO DE
          lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1    ; ADDR OF BYTES IN MAT
; PC no jump -4236
          call    DCMP          ; (DE)-((HL))
          breq    MVAR1
; PC no branch -4478
          brcs    PC+3
          jmp    OBERR         ; TOO BIG FOR ORIGINAL DIMENSIONS
;*   
;*  SET NEW DIMENSIONS
;*   
MVAR1:    lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1
          adiw    ZL,1          ; PASS BYTES IN MAT
          adiw    ZL,1
          pop     XH
          pop     XL            ; MOVE COL
          pop     YH
          pop     YL            ; GET ROW INTO DE
          push    XL
          push    XH
; PC no jump -4140
          _DSTOR
          pop     YH
          pop     YL            ; GET COL INTO DE
; PC no jump -4144
          _DSTOR
;*
; PC no jump -3743
          call    EATRP
MVAR0:    lds     ZL,TEMP1     ;LHLD
          lds     ZH,TEMP1+1    ; VAR DEFINITION ADDR L_3895
;*   
;*   
;*  SAVE VAR INFO IN STATIC AREA
;*   
          _XCHG                 ; VAR DEFINITION ADDRESS TO DE, SHIFT TO HL
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF STATIC AREA, SHIFT TO STACK
; PC no jump -4155
          _DSTOR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; SHIFT TO HL, ADDR OF STATIC TO STACK
          lds     ZL,TEMP2     ;LHLD
          lds     ZH,TEMP2+1    ; TEMP2  GET SIZE IN BYTES L_389B
          _XCHG                 ; VAR ADDR TO HL, SIZE TO DE
          adiw    ZL,1          ; SKIP SIZE IN VAR DEFINITION
          adiw    ZL,1
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF STATIC AREA
; PC no jump -4166
          _DSTOR         ; SAVE SIZE
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF VAR
; PC no jump -4165
          _DLOAD         ; GET FIRST INDEX
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF STATIC
; PC no jump -4174
          _DSTOR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF VAR
; PC no jump -4173
          _DLOAD         ; GET SECOND INDEX
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF STATIC
; PC no jump -4182
          _DSTOR
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF VAR
; PC no jump -4181
          _DLOAD         ; MUST BE ZERO FOR END OF INDEX LIST
          mov     A,YH
          or      A,YL
; PC no branch -887
          breq    PC+2
          rjmp    MDERR         ; MAT DIM ERROR
          _XCHG                 ; ADDR OF DATA TO DE
          movw    DPL,ZL     ; XTHL
          pop     ZH
          pop     ZL
          push    DPL
          push    DPH           ; ADDR OF STATIC
; PC no jump -4196
          _DSTOR
          pop     YH
          pop     YL            ; CLEAN STACK OFF
          ret
;*
;*
;MA:       .byte 2             ; ADDR OF DEFINITION
;MABZ:     .byte 2             ; SIZE IN BYTES
;MAROW:    .byte 2             ; ROW SIZE IN ELEMENTS
;MACOL:    .byte 2             ; COL SIZE IN ELEMENTS
;MADB:     .byte 2             ; DATA BASE ADDRESS
;*
;MR:       .byte 2
;MRBZ:     .byte 2
;MRROW:    .byte 2
;MRCOL:    .byte 2
;MRDB:     .byte 2
;*
;MB:       .byte 2
;MBBZ:     .byte 2
;MBROW:    .byte 2
;MBCOL:    .byte 2
;MBDB:     .byte 2
;*
;*
;% end of file BSM#MTC3
;*
;*     THE  END
;*
;XEND:     .byte 1
;*
;*
;*     THE  INITIALIZATION  CODE
;*
;*
;start of file BSM#INIT
;end of file BSM#INIT
;*
;*
;*     THE  ONCE  ONLY  CODE
;*
;*
;*
;*    ONCE ONLY CODE TO PREPARE BASIC FOR PRODUCTION
;*
;*
;start of file BSM#ONCE

cFOPEN:
cFCLOS:
cRDBYT:
cWRBYT:
cRDBLK:
cWRBLK:

ret

; IF SOLOS
cRETRN:			; BYE  FOR SOLOS
	; this should create a blue screen of death
	
	ldi ARGL,low(ILI9341_BLUE)
	ldi ARGH,high(ILI9341_BLUE)
	
	ldi XL,0
	ldi XH,0
	ldi c_tmp,low(ILI9341_TFTWIDTH)
	ldi idx,high(ILI9341_TFTWIDTH)
	
	ldi YL,0
	ldi YH,0
	ldi r24,low(ILI9341_TFTHEIGHT)
	ldi r25,high(ILI9341_TFTHEIGHT)
		
	rcall fillRect




forever:
	wdr
	rjmp forever

;---------------------------------------------------------------

// fill a rectangle
fillRect:
;	(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 

  // rudimentary clipping (drawChar w/big text requires this)
;  if((x >= _width) || (y >= _height)) return;
;  if((x + w - 1) >= _width)  w = _width  - x;
;  if((y + h - 1) >= _height) h = _height - y;

	push ARGL		; shadows color
	push ARGH
	push c_tmp		; shadows height
	push idx
	push r24		; shadows witdth
	push r25

	mov ARGL,r24	; save width
	mov ARGH,r25

 	mov r24,XL		; move position to start
	mov r25,YH

	add XL,ARGL
	adc XH,ARGH
	sbiw X,1			; (x, y, x+w-1, y);

;	mov ARGL,c_tmp
;	mov ARGH,idx

	mov ARGL,YL
	mov ARGH,YH

	add YL,c_tmp
	adc YH,idx
	sbiw Y,1			; (x, y, x+w-1, y+h-1);
	cbi SS_PORT,SS			; chip select

	call setAddrWindow

	; restore count values
	pop r25
	pop r24
	pop idx
	pop c_tmp

;  uint8_t hi = color >> 8, lo = color;

	pop ARGH
	pop ARGL

	
l_870:	
	push r24
	push r25
l_873:

	mov oDATA,ARGH			; spiwrite(color >> 8);
	call writedata
	mov oDATA,ARGL			; spiwrite(color);
	call writedata

	sbiw r24,1
	brne l_873
	pop r25
	pop r24
	subi c_tmp,1
	sbc idx,zero
	brne l_870
	
	
	sbi SS_PORT,SS			; chip select
	
	ret

;********************
;*	CONTRL	    *
;********************
;Control character processing
TERMCTL:
	lds c_tmp,TINCOD
	cpi c_tmp,0x1B
	brne TERMCTL_10

	; currently only code 4 (return cursor) is active
	mov XL,r10
	mov XH,r11

	clr c_tmp
	sts TINCOD,c_tmp
	ret

TERMCTL_10:
;	DJMP	D1			; directed jump on control code
	mov DPL,ZL
	mov DPH,ZH			; save Z where we can recover it
	;ANDW	#17,D1		; strip trash from character
;	andi A,0x0F
	mov c_tmp,XH
	lsl c_tmp			; multiply by 2 for offset
	;DJMP	D1			; directed jump on function code
	ldi ZL,low(CONTRL_T*2)
	ldi ZH,high(CONTRL_T*2)
	add ZL,c_tmp
	adc ZH,zero
	lpm 
	mov c_tmp,r0
	adiw Z,1
	lpm
	mov ZH,r0
	mov ZL,c_tmp
	ijmp


TLFT:
	; move hardware cursor 1 space left
	cp r10,zero
	breq TLFT_99	; already at left edge of screen
	push YL
	push YH
	rcall setHWcurs

	ld XH,Y
	andi  XH,0x7F
	st Y,XH
	rcall drawChar

	dec r10
	rcall setHWcurs

	ld XH,Y
	ori  XH,0x80
	st Y,XH
	rcall drawChar
	pop YH
	pop YL

TLFT_99:
	; restore Z
	mov ZL,DPL
	mov ZH,DPH
	ret

TLF:
	; move hardware cursor 1 space down
	; scroll the display if needed

	push YL
	push YH
	push XH
	rcall setHWcurs

	ld XH,Y
	andi  XH,0x7F
	st Y,XH
	rcall drawChar

	ldi c_tmp,15
	cp r11,c_tmp
	brge TLF_10

	inc r11
	ldi c_tmp,0x0F
	and r11,c_tmp

;	cp r12,zero
;	brne TLF_10

	rjmp TLF_99

TLF_10:
	; hardware scroll the display
	; r12 is the scroll offset in lines
	


    ; r12 is the scroll offset in lines
	dec r12	; set HW scroll
	ldi XH,0x0F
	and r12,XH

	; erase the line at the scroll point
	rcall setHWcurs
	ldi ZH,40
	ldi ZL,' '
TLF_15:
	st Y+,ZL
	dec ZH
	brne TLF_15

;	brne TLF_20

;	clr r11	
;TLF_20:
	rcall refreshScreen


TLF_99:

	;draw new cursor
	rcall setHWcurs

	ld XH,Y
	ori  XH,0x80
	st Y,XH
	rcall drawChar
	pop XH
	pop YH
	pop YL
	; restore Z
	mov ZL,DPL
	mov ZH,DPH
	ret

TCR:
	; home hardware cursor left
	; move hardware cursor 1 space down

	push YL
	push YH
	push XH		; save the char
	rcall setHWcurs

	ld XH,Y
	andi  XH,0x7F
	st Y,XH
	rcall drawChar

	; erase to end of line
	; alternative is to split line
	ldi idx,40
	sub idx,r10
TCR_10:
	ldi  XH,' '
	rcall setHWcurs
	st Y,XH
	push idx
	rcall drawChar
	pop idx
	inc r10
	dec idx
	brne TCR_10
	
	clr r10

	rcall setHWcurs

	ld XH,Y
	ori  XH,0x80
	st Y,XH
	rcall drawChar
	pop XH
	pop YH
	pop YL
TCR_99:
	; restore Z
	mov ZL,DPL
	mov ZH,DPH
	ret

TRIT:
	; move hardware cursor 1 space right
	ldi c_tmp,60
	cp r10,c_tmp
	brge TLF_99
	push YL
	push YH
	rcall setHWcurs

	ld XH,Y
	andi  XH,0x7F
	st Y,XH
	rcall drawChar

	inc r10
	ldi c_tmp,0x3F
	and r10,c_tmp

	rcall setHWcurs

	ld XH,Y
	ori  XH,0x80
	st Y,XH
	rcall drawChar
	pop YH
	pop YL

TRIT_99:
	; restore Z
	mov ZL,DPL
	mov ZH,DPH
	ret

TESC:
	; process escape codes 1B
	sts TINCOD,XH
	mov ZL,DPL
	mov ZH,DPH
	ret

CURLFT:
CURRGH:
;CURRGT: ; TRIT
IGNORE:
	mov ZL,DPL
	mov ZH,DPH
	ret


setHWcurs:
	
	; there are 16 lines so multiply by 60 to get line offset
	ldi YL,low(CC00)
	ldi YH,high(CC00)	; this is the fake pointer address from the VDM used here for 
					; sentiment

	mov r13,r11
	sub r13,r12		; scroll offset
	ldi ARGL,0x0F
	and r13,ARGL

	ldi ARGL,40
	mul ARGL,r13

	
	add YL,r0
	adc YH,r1	

	add YL,r10		; add in the column
	adc YH,zero

	ret

cSYSOT:
cAOUT:
drawSingle:
	; pass char in reg A and B (for now)

	;r10  cursor postion for column
	;r11  cursor position for line
	andi  XH,0x7F

	cpi  XH,' '		; process non printable terminal chars
	brge OF2_10
	rjmp TERMCTL

OF2_10:

	push XL
	push XH
	push YL
	push YH		; save Y points to input buffer	
	push ZL
	push ZH

	rcall setHWcurs
		
	st Y,XH			; save it to VDM ram for redraws
	
	rcall drawChar  ; draw the char at the cursor

	; update the hardware cursor
	inc r10
	
	ldi c_tmp,40
	cp r10,c_tmp
	brlt OF2_20
	clr r10
	inc r11
	ldi c_tmp,16
	cp r11,c_tmp
	brlt OF2_20
	clr r11			; could scroll display here
OF2_20:	
	push XH
	ldi XH,0xA0		; inverse space
	rcall setHWcurs
	st Y,XH
	rcall drawChar

	pop XH
	pop ZH
	pop ZL
	pop YH
	pop YL
	pop XH
	pop XL
	ret



; ENDF


refreshScreen:
	; the ILI9341 can only display 15 pixel high chars
	; so we only get 40 chars in the column and retain
	; 16 lines.
	; (could use a smaller font, then we would eat
	; most of the extra ram for the display


	push r10
	push r11		; save current HW cursor

	; refreshes screen from bottom up	right to left

	ldi A,15
	mov r11,A		; temp counter for outer loop

	ldi A,39		; the number of columns we can write
	mov r10,A		; use a temp counter so we do not have to save regiters

	rcall setHWcurs ; we want to refresh as fast as possible

refresh0:
	; this refreshes a line				
	ldi A,39		; the number of columns we can write
	mov r10,A		; use a temp counter so we do not have to save regiters

refresh1:	
	
	rcall drawChar	; this will put the character indexed in SRAM
					; onto the display
						
	sbiw Y,1		; next char
	; Y needs to wrap
	ldi c_tmp,high(CC00)
	cpi YL,low(CC00)
	cpc YH,c_tmp
	brcc refresh2
	ldi YL,low(RAMEND)
	ldi YH,high(RAMEND)
refresh2:
	dec r10
	brpl refresh1

	dec r11
	brpl refresh0	; next line
		
	pop r11
	pop r10

	ret;


drawChar:
	; re-draws a buffered character

	;the function setHWcurs should be called to set the Y
	;pointer and the cursor location
	 
	;the char should be already stored in the display bufer
	;this is the refresh update

	; Y pair should point to SRAM (cursor position)
	; Z pair used to load character rom pattern
	push  XL
	push  XH


	ld  XH,Y					; be consistant with the main program

	ldi ZL,low(CG6574*2)
	ldi ZH,high(CG6574*2)
	
	; if high bit set char is reverse video or special function
	; such as a box draw
	ldi ARGL,low(ILI9341_BLACK)
	ldi ARGH,high(ILI9341_BLACK)
	sts bgColor,ARGL
	sts bgColor+1,ARGH
	ldi ARGL,low(ILI9341_GREEN)
	ldi ARGH,high(ILI9341_GREEN)
	sts fgColor,ARGL
	sts fgColor+1,ARGH
	tst  XH
	brpl hascolor

	ldi ARGL,low(ILI9341_BLACK)
	ldi ARGH,high(ILI9341_BLACK)
	sts fgColor,ARGL
	sts fgColor+1,ARGH
	ldi ARGL,low(ILI9341_GREEN)
	ldi ARGH,high(ILI9341_GREEN)
	sts bgColor,ARGL
	sts bgColor+1,ARGH


hascolor:
	andi  XH,0x7F		; remove high bit if not needed
	mov A, XH			; save index

	; we need to multiply this index by pixel height
	ldi XH,12

	mul A,XH
	
	; now add it into the character generator to get offset	
	add ZL,r0
	adc ZH,r1

cellloop:	
	
	; setup drawing window
	push YL
	push YH
	
	; calculate column address

	mov XL,r10
	ldi XH,8		; our cell width is 8
	mul XL,XH
	mov XL,r0
	mov XH,r1	
	
	; adjust char line for scroll offset
	
	mov YL,r11		
;	sub YL,r12		; add in the scroller offset
;	andi YL,0x0F    ; line can not be greater than 15 (16 lines)
;	cpI YL,0xF0
;	brne noScrol
;	mov YL,r13
;noScrol:
	ldi YH,15		; our cell height is 15
	mul YL,YH
	mov YL,r0
	mov YH,r1

	mov r24,XL		; starting offset for drawing window
	mov r25,XH
	
	adiw X,7		; 8 pixels in a window 0 to 7

	mov ARGL,YL		; set window start
	mov ARGH,YH
	
	adiw Y,14		; cell is 15 pixels high 0 to 14

;cli
	cbi SS_PORT,SS					; chip select

	call setAddrWindow ;r24,r25,X, argl,argh,Y
	
	pop YH
	pop YL		; restore Y address for next char

	; first row of cell is background
	; draw line 0 the lead slug
	ldi idx,8
cell1:
	; write pixel
	lds oDATA,bgColor+1			; spiwrite(color >> 8);
	call writedata
	lds oDATA,bgColor			; spiwrite(color);
	call writedata
	dec idx
	brne cell1

	ldi c_tmp,12					; 12 lines in the cell
cell3: 

	lpm							; r0 now contains the bitmask  
	adiw Z,1

	mov A,r0
	ldi idx, 8	; there are 7 bits in the mask plus a blank for separation
				; the real system had 9 bits in the cell with two lines between

	
	rol	A		; preshift the bitmask	
charloop:
	; set color 1
	lds ARGL,bgColor
	lds ARGH,bgColor+1
	brcc markColorset
	; set color 2
	lds ARGL,fgColor
	lds ARGH,fgColor+1
markColorset:
	; write pixel
	mov oDATA,ARGH			; spiwrite(color >> 8);
	call writedata
	mov oDATA,ARGL			; spiwrite(color);
	call writedata
	rol A
	dec idx
	brne charloop
	
	dec c_tmp
	brne cell3

	; two slugs of lead between the lines to bring us to 15

celldone:
.if 1
	ldi c_tmp,2				; expand by 2 this is line 0 and 1

cell19:
	ldi idx,8
cell20:
	; write pixel
	mov oDATA,zero			; spiwrite(color >> 8);
	call writedata
	mov oDATA,zero			; spiwrite(color);
	call writedata
	dec idx
	brne cell20
	dec c_tmp
	brne cell19
.endif	

cell99:
	; char should now be drawn
	sbi SS_PORT,SS					; chip select

;sei	
	pop  XH
	pop  XL	
	ret


; ------------------------------------------------------------------
writecommand:
	cbi DC_PORT,DC					; set command bit or bit 9

	; could save bus config here (in case more than one 
	; device is active)
	; wait for any pending bus activity
	rjmp writedata8 ; same as data

writecommand_BB:
	push idx

	; bitbang SPI DBI type C - Option 1 (3 wire)
	
	; interface writes are on rising transition of sclock

	; The byte to send is preceded by a DCX bit that determines if
	; the data is a command or a parameter  This bit is in carry

	ldi idx,SR_OUTPUTS		; number of bit to transmit + carry

	clc						; bit 9 is clear for command
	rjmp dlooper			; rest is same

; ------

writedata:
	sbi DC_PORT,DC					; or bit 9

	; could save bus config here (in case more than one 
	; device is active)
	; wait for any pending bus activity
writedata8:
	out SPDR, oData
	nop

waitData:
	in r9,SPSR
	sbrs r9,SPIF
	rjmp waitData



	; leave the DC line in common state

	ret

writedata_BB:
	push idx
	; bitbang SPI DBI type C - Option 1 (3 wire)
	
	; interface writes are on rising transition of sclock

	; The byte to send is preceded by a DCX bit that determines if
	; the data is a command or a parameter  This bit is in carry

	ldi idx,SR_OUTPUTS		; number of bit to transmit + carry

	sec						; bit 9 is set for data
	
dlooper:
	cbi SR_DATA,SR_OUT		; pre clear the data line
	brcc l1789				; skip if carry set
	sbi SR_DATA,SR_OUT		
l1789:
	cbi SR_PORT,SR_SCL		; drop the clock for the next go-round
	lsl oDATA					; use setup time to shift the next bit into carry
	sbi SR_PORT,SR_SCL		; raise the clock to latch the bit
	dec idx
	brne dlooper
	
	pop idx	
	ret


begin: 
	; Using hardware SPI
	; SCL and MOSI are set in init to outputs
	; DC line is on pd 6 (on the sheild)
	; SS line is on pd 5 (on the sheild)

	ldi A,(1<<SPE)|(1<<MSTR)
	out SPCR,A

	ldi A,1<<SPI2X
	out SPSR,A		; set fast as possible f/2


;    rcall Serial_println
;    .db "linux HX8357B",0x00
    
	; reset the TFT
	sbi PORT_RSTTFT,RSTTFT
    ldi r16,low(100)
    ldi r17,high(100)
    call delay

	cbi PORT_RSTTFT,RSTTFT    
    ldi r16,low(100)
    ldi r17,high(100)
    call delay

 	sbi PORT_RSTTFT,RSTTFT
    ldi r16,low(150)
    ldi r17,high(150)				; give the display time to reset
    call delay

	cbi SS_PORT,SS					; enable chip
	

;    ldi r16,low(500)
;    ldi r17,high(500)
;    call delay

;	ldi oData,0x01
;    rcall writecommand

;    ldi r16,low(200)
;    ldi r17,high(200)
;    call delay

;		seqpower
	ldi ZL,low(ILI9341_setup*2)
	ldi ZH,high(ILI9341_setup*2)

seq_on:
	lpm
	cp r0,zero
	brne PC+2
	rjmp seq_exit
	adiw ZL,1
	mov idx,r0
	lpm
	adiw ZL,1
	mov oDATA,r0
	rcall writecommand
	dec idx
seq_send:
	lpm
	adiw ZL,1
	mov oDATA,r0
	rcall writedata
	dec idx
	brne seq_send
	rjmp seq_on

seq_exit:
	ldi oDATA,ILI9341_SLPOUT
	rcall writecommand

    ldi r16,low(120)
    ldi r17,high(120)
    call delayM

;		main screen turn on
    ldi oDATA,ILI9341_DISPON
	rcall writecommand

	sbi SS_PORT,SS					; disable chip select

    ldi r16,low(10)
    ldi r17,high(10)
    call delay
	
	ret  

setAddrWindow: 
	//(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 

	// use processor X and Y registers to facilitate
	// naming  

	ldi oDATA,ILI9341_CASET			; // Column addr set
	rcall writecommand;
	mov oDATA,r25
	rcall writedata
	mov oDATA,r24					; // XSTART 
	rcall writedata
	mov oDATA,XH
	rcall writedata
	mov oDATA,XL
	rcall writedata				;     // XEND

	ldi oDATA,ILI9341_PASET			; // Row addr set
	rcall writecommand;
	mov oDATA,ARGH
	rcall writedata
	mov oDATA,ARGL						; /Y XSTART 
	rcall writedata
	mov oDATA,YH
	rcall writedata
	mov oDATA,YL
	rcall writedata				;     // YEND

	ldi oDATA,ILI9341_RAMWR			; write to RAM
	rcall writecommand;


	ret

;	----------------------------------------------------------------------

init:

;	clr P1.1					; clear status LED 				


	; drive the port directions here
	in ARGL,SR_PORT
	ori ARGL, (1<<DD_SCL) | (1 << DD_MOSI)
	out SR_PORT,ARGL

	in	ARGL,DD_DATA
	ori ARGL, (1<<DD_SCL) | (1 << DD_MOSI)
	out DD_DATA,ARGL

	sbi SS_DD,DD_SS					; chip select
	sbi SS_PORT,SS					; chip select

	sbi DDRB,DDB2
	sbi PORTB,PINB2

	sbi DC_DD,DD_DC					; data command
	sbi DC_PORT,DC					; or bit 9

	sbi DD_RSTTFT,RSTTFT_DD
	sbi PORT_RSTTFT,RSTTFT

; port C is nominally used for addressing the board
; alternativly it can control I2C or a LCD character display
	ldi r24,0x3F
	out PORTC,r24

	sbi DDRD,DDD3				; this is now the keyboard wakeup line (RTS)

	nop
	nop							; let the pins settle	
	nop

	clr zero


	ldi ZL,low(SRAM_START)		; fix for bad init when POD table is not set
	ldi ZH,high(SRAM_START)
	add ZL,DPL
	adc ZH,DPH
	st Z,r24				

	sts ASSR,zero		; system clock source
	sts TCCR2B,zero		; stop timer

	; could check async state flags here if needed

	cli

; setup timer2 for us delays
	ldi r24,25
	sts OCR2A,r24

	; set CTC mode 
	ldi r24, 1 << WGM21
	sts TCCR2A,r24


	; start timer
	ldi r24, 4 << CS20  ; clock/64
	sts TCCR2B,r24

	
; setup serial backchannel -- useful for debugging

	ldi r24,low(kBaud)
	ldi r25,high(kBaud)

	sts	UBRR0H,r25		; set the Baud divisor
	sts	UBRR0L,r24

	ldi	r24,1<<U2X0		; Double speed
	sts	UCSR0A,r24		

						; enable transmitter incase we want to write something
	lds r24,UCSR0B
	andi r24, ~(1<<RXCIE0) & ~(1<<TXCIE0) &  ~(1<<TXEN0) & ~(1<<RXEN0) & ~(1<<UCSZ02) 
	ori r24, (1<<RXCIE0) | (1<<TXEN0) |(1<<RXEN0) 
	sts UCSR0B,r24

	lds r24,UCSR0C
	ori r24, (1<<UCSZ01) | (1<<UCSZ00)
	sts UCSR0C,r24


	ldi r24,1 << OCIE2A
	sts TIMSK2,r24	; used for delay timer

	; this is a reset timer for the keyboard
	; the keyboard goes to sleep after a few minutes of inacivity
	ldi r19,48				; should be about 5 minutes or so


	sei
	ret



alt_ILI9341_setup:
    .db	4, 0xCF,0x00,0x8B,0x30,\
		5, 0xED,0x67,0x03,0x12,0x81,\
		4, 0xE8,0x85,0x10,0x7A,\
		6, 0xCB,0x39,0x2C,0x00,0x34,0x02,\
		2, 0xF7,0x20,\
		3, 0xEA,0x00,0x00
	.db	2, ILI9341_PWCTR1,0x1B,\
		2, ILI9341_PWCTR2,0x10,\
		3, ILI9341_VMCTR1,0x3F,0x3C
	.db	2, ILI9341_VMCTR2,0xB7,\
		2, ILI9341_MADCTL,0x08,\
		2, ILI9341_PIXFMT,0x55,\
		3, 0xB1,0x00,0x1B,\
		3, 0xB6,0x0A,0xA2,\
		2, 0xF2,0x00
	.db	2, 0x26,0x01,\
		16,0xE0,0x0F
    .db 0x2A,0x28,0x08,0x0E,0x08,0x54,0xA9,0x43
	.db 0x0A,0x0F,0x00,0x00,0x00,0x00

	.db 16,0xE1,0x00,0x15
	.db 0x17,0x07,0x11,0x06,0x2B,0x56,0x3C,0x05
	.db 0x10,0x0F,0x3F,0x3F,0x0F,0
 

ILI9341_setup:
	.db 4, 0xEF, 0x03, 0x80, 0x02,\
		4, 0xCF, 0x00, 0XC1, 0X30
	.db	5, 0xED, 0x64, 0x03, 0X12, 0X81,\
		4, 0xE8, 0x85, 0x00, 0x78,\
		6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02
	.db	2, 0xF7, 0x20,\
		3, 0xEA, 0x00, 0x00,\
		2, ILI9341_PWCTR1, 0x23,\
		2, ILI9341_PWCTR2, 0x10,\
		3, ILI9341_VMCTR1, 0x3e, 0x28,\
		2, ILI9341_VMCTR2, 0x86
	.db	2, ILI9341_MADCTL, 0x28,\
		2, ILI9341_PIXFMT, 0x55
	.db	3, ILI9341_FRMCTR1, 0x00, 0x18
	.db	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27,\
		2, 0xF2, 0x00
	.db	2, ILI9341_GAMMASET, 0x01,\
		16,ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,\
		   0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,\
		16,ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,\
		   0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,\
		0

;	.db 5,ILI9341_PASET,0x00,0x00,0x00,0xFE
;	.db 5,ILI9341_CASET,0x00,0x00,0x01,0x3F


CG6574:
	.db 0xFE, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
	.db 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x00, 0x00, 0x00
	.db 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00
	.db 0x80, 0x40, 0x20, 0x10, 0xF8, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0xC6, 0xAA, 0x92, 0xAA, 0xC6, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0x00, 0x02, 0x04, 0x08, 0x90, 0xA0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0x82, 0xFE, 0x28, 0x28, 0xEE, 0x00, 0x00, 0x00
	.db 0x20, 0x40, 0xF8, 0x44, 0x22, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x08, 0x04, 0xFE, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0xFE, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x10, 0x10, 0x92, 0x54, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0x10, 0x10, 0x54, 0x38, 0x10, 0x92, 0x54, 0x38, 0x10, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x20, 0x40, 0xFE, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0xC6, 0xAA, 0x92, 0xAA, 0xC6, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0xC6, 0x82, 0x92, 0x82, 0xC6, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0x82, 0x82, 0xFE, 0x82, 0x82, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0x38, 0x54, 0x92, 0x92, 0x9E, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0x9E, 0x92, 0x92, 0x54, 0x38, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0xF2, 0x92, 0x92, 0x54, 0x38, 0x00, 0x00, 0x00
	.db 0x38, 0x54, 0x92, 0x92, 0xF2, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0x00, 0x22, 0x14, 0x08, 0x94, 0xA2, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00
	.db 0x7C, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0xC6, 0x00, 0x00, 0x00
	.db 0x02, 0x02, 0x02, 0x02, 0xFE, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0x10, 0x10, 0x10, 0x38, 0x38, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00
	.db 0x78, 0x84, 0x84, 0x80, 0x60, 0x10, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0xFE, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0xFE, 0x92, 0x92, 0x92, 0xF2, 0x82, 0x82, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0x82, 0x82, 0xF2, 0x92, 0x92, 0x92, 0xFE, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0x82, 0x82, 0x9E, 0x92, 0x92, 0x92, 0xFE, 0x00, 0x00, 0x00
	.db 0xFE, 0x92, 0x92, 0x92, 0x9E, 0x82, 0x82, 0x82, 0xFE, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00
	.db 0x48, 0x48, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x28, 0x28, 0x28, 0xFE, 0x28, 0xFE, 0x28, 0x28, 0x28, 0x00, 0x00, 0x00
	.db 0x10, 0x7E, 0x90, 0x90, 0x7C, 0x12, 0x12, 0xFC, 0x10, 0x00, 0x00, 0x00
	.db 0x40, 0xA2, 0x44, 0x08, 0x10, 0x20, 0x44, 0x8A, 0x04, 0x00, 0x00, 0x00
	.db 0x70, 0x88, 0x88, 0x50, 0x20, 0x52, 0x8C, 0x8C, 0x72, 0x00, 0x00, 0x00
	.db 0x18, 0x18, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x08, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00
	.db 0x20, 0x10, 0x08, 0x08, 0x08, 0x08, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x92, 0x54, 0x38, 0x54, 0x92, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x10, 0x10, 0xFE, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x20, 0x40, 0x00
	.db 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00
	.db 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x86, 0x8A, 0x92, 0xA2, 0xC2, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0x10, 0x30, 0x50, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x02, 0x04, 0x38, 0x40, 0x80, 0x80, 0xFE, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x02, 0x02, 0x3C, 0x02, 0x02, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0x04, 0x0C, 0x14, 0x24, 0x44, 0x84, 0xFE, 0x04, 0x04, 0x00, 0x00, 0x00
	.db 0xFE, 0x80, 0x80, 0xF8, 0x04, 0x02, 0x02, 0x84, 0x78, 0x00, 0x00, 0x00
	.db 0x3C, 0x40, 0x80, 0x80, 0xFC, 0x82, 0x82, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0xFE, 0x82, 0x04, 0x08, 0x10, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x82, 0x82, 0x7C, 0x82, 0x82, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x82, 0x82, 0x7E, 0x02, 0x02, 0x04, 0x78, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x20, 0x40, 0x00
	.db 0x08, 0x10, 0x20, 0x40, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x7C, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x20, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00
	.db 0x3C, 0x42, 0x42, 0x02, 0x0C, 0x10, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00
	.db 0x3C, 0x42, 0x9A, 0xAA, 0xAA, 0xBC, 0x80, 0x40, 0x3C, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0x82, 0xFE, 0x82, 0x82, 0x82, 0x00, 0x00, 0x00
	.db 0xFC, 0x42, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x42, 0xFC, 0x00, 0x00, 0x00
	.db 0x3C, 0x42, 0x80, 0x80, 0x80, 0x80, 0x80, 0x42, 0x3C, 0x00, 0x00, 0x00
	.db 0xF8, 0x44, 0x42, 0x42, 0x42, 0x42, 0x42, 0x44, 0xF8, 0x00, 0x00, 0x00
	.db 0xFE, 0x80, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0xFE, 0x00, 0x00, 0x00
	.db 0xFE, 0x80, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
	.db 0x3C, 0x42, 0x80, 0x80, 0x80, 0x9E, 0x82, 0x42, 0x3C, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x82, 0x82, 0xFE, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00, 0x00
	.db 0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00, 0x00
	.db 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x00
	.db 0x82, 0x84, 0x88, 0x90, 0xA0, 0xD0, 0x88, 0x84, 0x82, 0x00, 0x00, 0x00
	.db 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFE, 0x00, 0x00, 0x00
	.db 0x82, 0xC6, 0xAA, 0x92, 0x92, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00, 0x00
	.db 0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x82, 0x82, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00, 0x00
	.db 0xFC, 0x82, 0x82, 0x82, 0xFC, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
	.db 0x38, 0x44, 0x82, 0x82, 0x82, 0x92, 0x8A, 0x44, 0x3A, 0x00, 0x00, 0x00
	.db 0xFC, 0x82, 0x82, 0x82, 0xFC, 0x90, 0x88, 0x84, 0x82, 0x00, 0x00, 0x00
	.db 0x7C, 0x82, 0x80, 0x80, 0x7C, 0x02, 0x02, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x7C, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x10, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x82, 0x82, 0x92, 0x92, 0xAA, 0xC6, 0x82, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x82, 0x00, 0x00, 0x00
	.db 0x82, 0x82, 0x44, 0x28, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00
	.db 0xFE, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFE, 0x00, 0x00, 0x00
	.db 0x78, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x78, 0x00, 0x00, 0x00
	.db 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00
	.db 0x78, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x78, 0x00, 0x00, 0x00
	.db 0x10, 0x28, 0x44, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00
	.db 0x30, 0x30, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x78, 0x04, 0x7C, 0x84, 0x84, 0x7A, 0x00, 0x00, 0x00
	.db 0x80, 0x80, 0x80, 0xB8, 0xC4, 0x84, 0x84, 0xC4, 0xB8, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x78, 0x84, 0x80, 0x80, 0x84, 0x78, 0x00, 0x00, 0x00
	.db 0x04, 0x04, 0x04, 0x74, 0x8C, 0x84, 0x84, 0x8C, 0x74, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x78, 0x84, 0xFC, 0x80, 0x80, 0x78, 0x00, 0x00, 0x00
	.db 0x18, 0x24, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x74, 0x8C, 0x84, 0x8C, 0x74, 0x04, 0x04, 0x84, 0x78
	.db 0x80, 0x80, 0x80, 0xB8, 0xC4, 0x84, 0x84, 0x84, 0x84, 0x00, 0x00, 0x00
	.db 0x00, 0x10, 0x00, 0x30, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x44, 0x38
	.db 0x80, 0x80, 0x80, 0x88, 0x90, 0xA0, 0xD0, 0x88, 0x84, 0x00, 0x00, 0x00
	.db 0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0xEC, 0x92, 0x92, 0x92, 0x92, 0x92, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0xB8, 0xC4, 0x84, 0x84, 0x84, 0x84, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x78, 0x84, 0x84, 0x84, 0x84, 0x78, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0xB8, 0xC4, 0x84, 0x84, 0xC4, 0xB8, 0x80, 0x80, 0x80
	.db 0x00, 0x00, 0x00, 0x74, 0x8C, 0x84, 0x84, 0x8C, 0x74, 0x04, 0x04, 0x04
	.db 0x00, 0x00, 0x00, 0xB8, 0xC4, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x78, 0x84, 0x60, 0x18, 0x84, 0x78, 0x00, 0x00, 0x00
	.db 0x00, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x24, 0x18, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x84, 0x84, 0x84, 0x84, 0x8C, 0x74, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x82, 0x82, 0x82, 0x44, 0x28, 0x10, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x82, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x84, 0x48, 0x30, 0x30, 0x48, 0x84, 0x00, 0x00, 0x00
	.db 0x00, 0x00, 0x00, 0x84, 0x84, 0x84, 0x84, 0x8C, 0x74, 0x04, 0x84, 0x78
	.db 0x00, 0x00, 0x00, 0xFC, 0x08, 0x10, 0x20, 0x40, 0xFC, 0x00, 0x00, 0x00
	.db 0x1C, 0x20, 0x20, 0x20, 0x40, 0x20, 0x20, 0x20, 0x1C, 0x00, 0x00, 0x00
	.db 0x10, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00
	.db 0x30, 0x08, 0x08, 0x08, 0x04, 0x08, 0x08, 0x08, 0x30, 0x00, 0x00, 0x00
	.db 0x60, 0x92, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db 0x48, 0x92, 0x24, 0x48, 0x92, 0x24, 0x48, 0x92, 0x24, 0x00, 0x00, 0x00



; % define keyboard mapping tables

#define KEYMAP( \
        K00,K01,K02,K04,K05,K06,K07,K34,K35,K36,K30,K31,K32, \
    K19,K09,K0A,K0B,K0C,K0D,K0E,K3C,K3D,K3E,K3F,K38,K39,K3A,K33, \
    K18,K11,K12,K13,K14,K15,K16,K44,K45,K46,K47,K40,K41,    K3B, \
    K58,K03,K10,K2C,K2D,K2E,K2F,K4C,K4D,K4E,K48,K59,K49,    K42, \
    K1A,K22,K23,K08,        K17,K37,K0F,K4F,K50,K51,K52,K53,K4A  \
) \
.db KB_##K00, KB_##K01, KB_##K02, KB_##K03, KB_##K04, KB_##K05, KB_##K06, KB_##K07 \
.db KB_##K08, KB_##K09, KB_##K0A, KB_##K0B, KB_##K0C, KB_##K0D, KB_##K0E, KB_##K0F \
.db KB_##K10, KB_##K11, KB_##K12, KB_##K13, KB_##K14, KB_##K15, KB_##K16, KB_##K17 \
.db KB_##K18, KB_##K19, KB_##K1A, KB_NO,    KB_NO,    KB_NO,    KB_NO,    KB_NO    \
.db KB_NO,    KB_NO,    KB_##K22, KB_##K23, KB_NO,    KB_NO,    KB_NO,    KB_NO    \
.db KB_NO,    KB_NO,    KB_NO,    KB_NO,    KB_##K2C, KB_##K2D, KB_##K2E, KB_##K2F \
.db KB_##K30, KB_##K31, KB_##K32, KB_##K33, KB_##K34, KB_##K35, KB_##K36, KB_##K37 \
.db KB_##K38, KB_##K39, KB_##K3A, KB_##K3B, KB_##K3C, KB_##K3D, KB_##K3E, KB_##K3F \
.db KB_##K40, KB_##K41, KB_##K42, KB_NO,    KB_##K44, KB_##K45, KB_##K46, KB_##K47 \
.db KB_##K48, KB_##K49, KB_##K4A, KB_NO,    KB_##K4C, KB_##K4D, KB_##K4E, KB_##K4F \
.db KB_##K50, KB_##K51, KB_##K52, KB_##K53, KB_NO,    KB_NO,    KB_NO,    KB_NO    \
.db KB_##K58, KB_##K59, KB_NO,    KB_NO,    KB_NO,    KB_NO,    KB_NO,    KB_NO\
.db "123456789.123456789.123456789.12"

.equ KB_1 		=	0x31
.equ KB_2 		=	0x32
.equ KB_3 		=	0x33
.equ KB_4 		=	0x34
.equ KB_5 		=	0x35
.equ KB_6 		=	0x36
.equ KB_7 		=	0x37
.equ KB_8 		=	0x38
.equ KB_9 		=	0x39
.equ KB_0 		=	0x30
.equ KB_MINS	=	0x2C
.equ KB_EQL		=	0x3D
.equ KB_BSPC	=	0x08

.equ KB_TAB 	=	0x09
.equ KB_A 		=	0x41
.equ KB_B 		=	0x42
.equ KB_C 		=	0x43
.equ KB_D 		=	0x44
.equ KB_E 		=	0x45
.equ KB_F 		=	0x46
.equ KB_G 		=	0x47
.equ KB_H 		=	0x48
.equ KB_I 		=	0x49
.equ KB_J		=	0x4A
.equ KB_K		=	0x4B
.equ KB_L		=	0x4C

.equ KB_M		=	0x4D
.equ KB_N 		=	0x4E
.equ KB_O 		=	0x4F
.equ KB_P 		=	0x50
.equ KB_Q 		=	0x51
.equ KB_R 		=	0x52
.equ KB_S 		=	0x53
.equ KB_T 		=	0x54
.equ KB_U 		=	0x55
.equ KB_V 		=	0x56
.equ KB_W		=	0x57
.equ KB_X		=	0x58
.equ KB_Y		=	0x59
.equ KB_Z		=	0x5A

.equ KB_LCA	 	=	0x61
.equ KB_LCB 	=	0x62
.equ KB_LCC 	=	0x63
.equ KB_LCD 	=	0x64
.equ KB_LCE 	=	0x65
.equ KB_LCF 	=	0x66
.equ KB_LCG 	=	0x67
.equ KB_LCH 	=	0x68
.equ KB_LCI 	=	0x69
.equ KB_LCJ		=	0x6A
.equ KB_LCK		=	0x6B
.equ KB_LCL		=	0x6C

.equ KB_LCM		=	0x6D
.equ KB_LCN 	=	0x6E
.equ KB_LCO 	=	0x6F
.equ KB_LCP 	=	0x70
.equ KB_LCQ 	=	0x71
.equ KB_LCR 	=	0x72
.equ KB_LCS 	=	0x73
.equ KB_LCT 	=	0x74
.equ KB_LCU 	=	0x75
.equ KB_LCV 	=	0x76
.equ KB_LCW		=	0x77
.equ KB_LCX		=	0x78
.equ KB_LCY		=	0x79
.equ KB_LCZ		=	0x7A



.equ KB_SPC		=	0x20
.equ KB_LBRC	=	0x5B
.equ KB_RBRC	=	0x5D
.equ KB_BSLS	=	0x5C
.equ KB_SCLN	=	0x3B
.equ KB_QUOT	=	0x27
.equ KB_ENT		=	0x0D
.equ KB_COMM	=	0x2C
.equ KB_DOT		=	0x2E
.equ KB_SLSH	=	0x2F
.equ KB_GRV		=	0x60

; shifted symbols
.equ KB_BANG 	=	0x21
.equ KB_AT 		=	0x40
.equ KB_HASH	=	0x23
.equ KB_DOLR	=	0x24
.equ KB_PCNT 	=	0x25
.equ KB_CART 	=	0x5E
.equ KB_ETC		=	0x26
.equ KB_STAR	=	0x2A
.equ KB_LPAR	=	0x28
.equ KB_RPAR	=	0x29
.equ KB_USCR	=	0x5F
.equ KB_PLUS	=	0x2B


.equ KB_LBRA	=	0x7B
.equ KB_RBRA	=	0x7D
.equ KB_ORO		=	0x7C
.equ KB_CLLN	=	0x3A
.equ KB_DQOT	=	0x22
.equ KB_LT		=	0x3C
.equ KB_GT		=	0x3E
.equ KB_QMK		=	0x3F
.equ KB_TLDE	=	0x7E

.equ KB_HBRK	=	0xFF	; high break ^altdelete?
.equ KB_BRK		=	0x00	; the cuter break (mode select)
.equ KB_DEL		=	0x7F	; rubout

.equ KB_UP		=	0x17	; ^W for now
.equ KB_LEFT	=	0x01	; ^A for now
.equ KB_DOWN	=	KLF		; ^J for now
.equ KB_RGHT	=	KRIGHT	; ^L 

.equ KB_PGUP	=	0x04	; ^D for now
.equ KB_PLFT	=	0x12	; ^R for now
.equ KB_PGDN	=	0x06	; ^F for now
.equ KB_PGRT	=	0x0F	; ^O


.equ KB_FN1		=	0x84	; ESC | 80+0
.equ KB_FN2		=	0x88	; ESC | 80+1
.equ KB_FN3		=	0x8C	; ESC | 80+2
.equ KB_FN4		=	0x90	; ESC | 80+3
.equ KB_FN5		=	0x94	; ESC | 80+0
.equ KB_FN6		=	0x98	; ESC | 80+1
.equ KB_FN7		=	0x9C	; ESC | 80+2
.equ KB_FN8		=	0xA0	; ESC | 80+3
.equ KB_FN9		=	0xA4	; ESC | 80+0
.equ KB_FN10	=	0xA8	; ESC | 80+1
.equ KB_FN11	=	0xAC	; ESC | 80+2
.equ KB_FN12	=	0xB0	; ESC | 80+3
.equ KB_FN13	=	0xB4	; ESC | 80+0
.equ KB_FN14	=	0xB8	; ESC | 80+1
.equ KB_FN15	=	0xBC	; ESC | 80+2
.equ KB_NEW		=	0xC4	; new marker
.equ KB_RUN		=	0xC8	; loop marker
.equ KB_BRGT	=	0xCC	; brighness marker

.equ KB_NO		=	0x80	; invalid matrix location

; modifier key masks
.equ KB_LCTL	=	0xFC	; inverted offset to layer map
.equ KB_CAPS	=	0xFB
.equ KB_FN0		=	0xFD
.equ KB_LSFT	=	0xFE
.equ KB_RSFT	=	0xFE
.equ KB_LALT	=	0xFA	; same as option
.equ KB_LGUI	=	0xF9	; same as command
Layer0:
	; normal leyer
    /* Default Layer: plain keymap
     *      -----------------------------------------------------.
     *     |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bacpa|
     * |---------------------------------------------------------|.---.
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \||Fn1|
     * |---------------------------------------------------------||Fn2|
     * |CapsLk|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Enter ||Fn3|
     * |---------------------------------------------------------||Fn4|
     * |ShiftL  |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shift|Up|'---'
     * |---------------------------------------------------------|--.
     * |Ctrl |Fn0|Alt|GUI|   Space      | Sp|  `|Esc| Del |Lft|Dn|RT|
     * `---------------------------------------------------------'--'
     */
    KEYMAP(\
         1,   2,   3,   4,   5,   6,   7,   8,   9,   0,  MINS,EQL, BSPC,\
    TAB, LCQ, LCW, LCE, LCR, LCT, LCY, LCU, LCI, LCO, LCP,LBRC,RBRC,BSLS,FN1,\
    CAPS,LCA, LCS, LCD, LCF, LCG, LCH, LCJ, LCK, LCL,   SCLN, QUOT, ENT, FN2,\
    LSFT,LCZ, LCX, LCC, LCV, LCB, LCN, LCM, COMM, DOT,SLSH, RSFT,UP,	 FN3,\
    LCTL,FN0,LALT,LGUI,               SPC,SPC,GRV,BRK,DEL,LEFT,DOWN,RGHT,FN4 \
    )
Layer1:
	; shifted layer
   /* 
     *      -----------------------------------------------------.
     *     |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bacpa|
     * |---------------------------------------------------------|.---.
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \||Fn1|
     * |---------------------------------------------------------||Fn2|
     * |CapsLk|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Enter ||Fn3|
     * |---------------------------------------------------------||Fn4|
     * |ShiftL  |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shift|Up|'---'
     * |---------------------------------------------------------|--.
     * |Ctrl |Fn0|Alt|GUI|   Space      | Sp|  `|Esc| Del |Lft|Dn|RT|
     * `---------------------------------------------------------'--'
     */
    KEYMAP(\
         BANG,AT,  HASH,DOLR,PCNT,CART,ETC, STAR,LPAR,RPAR,USCR,PLUS, BSPC,\
    TAB, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,  LBRA,RBRA,ORO,  FN1,\
    CAPS,A,   S,   D,   F,   G,   H,   J,   K,   L,     CLLN, DQOT, ENT,  FN2,\
    LSFT,Z,   X,   C,   V,   B,   N,   M,   LT, GT,QMK, RSFT,UP,	      FN3,\
    LCTL,FN0,LALT,LGUI,               SPC,SPC,TLDE,BRK,DEL,LEFT,DOWN,RGHT,FN4 \
    )
Layer2:
	 ; Fn key layer
     ; special mapping for broken keyboard
	/* 
     *      -----------------------------------------------------.
     *     |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bacpa|
     * |---------------------------------------------------------|.---.
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \||Fn1|
     * |---------------------------------------------------------||Fn2|
     * |CapsLk|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Enter ||Fn3|
     * |---------------------------------------------------------||Fn4|
     * |ShiftL  |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shift|Up|'---'
     * |---------------------------------------------------------|--.
     * |Ctrl |Fn0|Alt|GUI|   Space      | Sp|  `|Esc| Del |Lft|Dn|RT|
     * `---------------------------------------------------------'--'
     */
    KEYMAP(\
         2,  FN2, AT, FN4,  FN5, FN6, 7,  8, 9,  FN10,  FN11 ,BRGT, BSPC,\
   TAB, Q,   Q,   W,   E,   R,   T,   4,   5,  6,   STAR,  LBRA,RBRA,ORO, FN12,\
    CAPS,A,   A,   S,   D,   F,   G,   1,   2,   3,   SLSH, DQOT, ENT,    FN13,\
    LSFT,TAB,   Z,   X,   C,   V,   B, PLUS,   0, MINS, EQL, RSFT,PGUP, FN14,\
    LCTL,FN0,LALT,RUN,               SPC,NEW,TLDE,BRK,HBRK,PLFT,PGDN,PGRT,FN15 \
    )
Layer3:
	; control key layer
    /* 
     *      -----------------------------------------------------.
     *     |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bacpa|
     * |---------------------------------------------------------|.---.
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \||Fn1|
     * |---------------------------------------------------------||Fn2|
     * |CapsLk|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Enter ||Fn3|
     * |---------------------------------------------------------||Fn4|
     * |ShiftL  |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shift|Up|'---'
     * |---------------------------------------------------------|--.
     * |Ctrl |Fn0|Alt|GUI|   Space      | Sp|  `|Esc| Del |Lft|Dn|RT|
     * `---------------------------------------------------------'--'
     */
    KEYMAP(\
    1&0xF,2&0xF,3&0xF,4&0xF,5&0xF,6&0xF,7&0xF,8&0xF,9&0xF,0&0xF,MINS&0xF,EQL&0x1F, BSPC,\
    TAB, Q&0x1F,W&0x1F,E&0xF,R&0x1F,T&0x1F,Y&0x1F,U&0x1F,I&0xF,O&0xF,P&0x1F,LBRC&0x1F,RBRC&0x1F,BSLS&0x1F,FN1,\
    CAPS,A&0xF,S&0x1F,D&0xF,F&0xF,G&0xF,H&0xF,J&0xF,K&0xF,L&0xF,SCLN&0x1F,QUOT&0xF,ENT, FN2,\
    LSFT,Z&0x1F,X&0x1F,C&0xF,V&0x1F,B&0xF,N&0xF,M&0xF,COMM&0xF,DOT&0xF,SLSH&0xF, RSFT,UP,	 FN3,\
    LCTL,FN0,LALT,LGUI,               SPC,SPC,GRV&0xF,BRK,DEL,LEFT,DOWN,RGHT,FN4 \
    )
Layer4:
	; caps lock layer
    /* 
     *      -----------------------------------------------------.
     *     |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bacpa|
     * |---------------------------------------------------------|.---.
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \||Fn1|
     * |---------------------------------------------------------||Fn2|
     * |CapsLk|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Enter ||Fn3|
     * |---------------------------------------------------------||Fn4|
     * |ShiftL  |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shift|Up|'---'
     * |---------------------------------------------------------|--.
     * |Ctrl |Fn0|Alt|GUI|   Space      | Sp|  `|Esc| Del |Lft|Dn|RT|
     * `---------------------------------------------------------'--'
     */
    KEYMAP(\
         1,   2,   3,   4,   5,   6,   7,   8,   9,   0,  MINS,EQL, BSPC,\
    TAB, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,  LBRC,RBRC,BSLS,FN1,\
    CAPS,A,   S,   D,   F,   G,   H,   J,   K,   L,     SCLN, QUOT, ENT, FN2,\
    LSFT,Z,   X,   C,   V,   B,   N,   M,   COMM, DOT,SLSH, RSFT,UP,	 FN3,\
    LCTL,FN0,LALT,LGUI,               SPC,SPC,GRV,BRK,DEL,LEFT,DOWN,RGHT,FN4 \
    )
	
; in the microcontroller applications Option(alt) and command(GUI) do
; not make much sense
	
	
.eseg
	.db 0

