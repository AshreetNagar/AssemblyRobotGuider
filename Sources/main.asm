*******************************************************************
* Final Project                                                   *
* Ashreet Nagar 500969636                                         *
* Saiharan Muraleethasan 500970443                                *
* Atheesh Thanaseelan 500962758                                   *
*                                                                 *
* COE538 Final Project                                            *
* Program to find a path through the maze,                        *
* then turn around and solve the maze                             *
*******************************************************************
; export symbols
*******************************************************************
              XDEF Entry, _Startup            ; export 'Entry' symbol
              ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
*******************************************************************
		INCLUDE 'derivative.inc' 

; equates section
*******************************************************************

L_ROT_INT       EQU 46 ; Time to rotate left (46 = 2 seconds)
R_ROT_INT       EQU 46 ; Time to rotate left (46 = 2 seconds)
ROT_180_INT     EQU 92 ; Time to rotate 180deg (92 should be equal to 4 seconds)

START          EQU 0
FWD            EQU 1
JUNC_DECIDER   EQU 2
BACKTRACK_JUNC EQU 3
Rotate_L       EQU 4
Rotate_R       EQU 5
Rotate_180     EQU 6
REVERSE        EQU 7
FWD_BACKTRACK  EQU 8
Solve_180      EQU 9
Solve_FWD      EQU 10
Solve_Turn     EQU 11
Solve_R        EQU 12

NULL            EQU 00

; variable section
*******************************************************************
              ORG $3850 ; Where our TOF counter register lives
TOF_COUNTER   dc.b 0 ; The timer, incremented at 23Hz
CRNT_STATE    dc.b 3 ; Current state register


T_L_ROT       ds.b 1 ; Left turn time
T_R_ROT       ds.b 1 ; Right turn time
T_180_ROT     ds.b 1 ; 180 degree turn time

;Sensor Variables
*******************************************************************
SENSOR_LINE     FCB $01             ; Storage for guider sensor readings
SENSOR_BOW      FCB $23             ; Initialized to test values
SENSOR_PORT     FCB $45
SENSOR_MID      FCB $67
SENSOR_STBD     FCB $89
SENSOR_NUM      RMB 1               ; The currently selected sensor
                FCB NULL            ; terminated by null

TEMP            RMB 1               ; Temporary location

THRESHOLD     EQU $7B ;Convert 123 to HEX: 7B
LINETHRESHOLD EQU $44 ;Convert  68 to HEX: 44

;Solution Storage Variables
*******************************************************************
JUNC0   FCB $01
JUNC1   FCB $FF
JUNC2   FCB $FF
JUNC3   FCB $FF
JUNC4   FCB $FF
JUNC5   FCB $FF
JUNC6   FCB $FF
JUNC7   FCB $FF
JUNC8   FCB $FF
JUNC9   FCB $FF

JUNCNUM FCB $00

HEADING FCB $01
TARGET_HEADING FCB $FF
 

; code section
*******************************************************************
              ORG   $4000 ; Where the code starts --------------------
Entry:                                                              ; |
_Startup:                                                           ; |
              CLI         ; Enable interrupts                         |
              LDS   #$4000 ; Initialize the stack pointer
                                                                    ; I
              BSET  DDRA,%00000011 ; STAR_DIR, PORT_DIR               N
              BSET  DDRT,%00110000 ; STAR_SPEED, PORT_SPEED           I
                
              JSR   INIT            ; Initialize ports
              JSR   openADC         ; Initialize the ATD
              JSR   ENABLE_TOF ; Jump to TOF initialization ----------

MAIN          JSR G_LEDS_ON       ; Enable the guider LEDs
              JSR READ_SENSORS    ; Read the 5 guider sensors
              JSR G_LEDS_OFF      ; Disable the guider LEDs
              LDY #6000           ; 300 ms delay to avoid
              JSR del_50us        ; display artifacts
              LDAA  CRNT_STATE
              JSR   DISPATCHER
              BRA   MAIN   

; data section
*******************************************************************
msg1          dc.b "Battery volt ",0
msg2          dc.b "State ",0
tab           dc.b "START ",0
              dc.b "FWD ",0
              dc.b "REV ",0
              dc.b "ALL_STP",0
              dc.b "FWD_TRN",0
              dc.b "REV_TRN",0

; subroutine section

;State dispatcher
*******************************************************************
DISPATCHER    CMPA  #FWD ; Else if it’s the FORWARD state               P
              BNE   NOT_FWD ;                                           A
              JSR   FWD_ST ; then call the FORWARD routine              T
              JMP   DISP_EXIT ; and exit                                C
                             ;                                           H
NOT_FWD       CMPA  #JUNC_DECIDER ; Else if it’s the REVERSE state            E
              BNE   NOT_JUNC_DECIDER ;                                        R
              JSR   JUNC_DECIDER_ST ; then call the REVERSE routine           |
              JMP   DISP_EXIT ; and exit                                |

NOT_JUNC_DECIDER    CMPA  #BACKTRACK_JUNC
                    BNE   NOT_BACKTRACK_JUNC ;                                           
                    JSR   BACKTRACK_JUNC_ST ; then call the REVERSE routine              
                    JMP   DISP_EXIT ; and exit                                

NOT_BACKTRACK_JUNC  CMPA  #Rotate_L
                    BNE   NOT_Rotate_L
                    JSR   Rotate_L_ST
                    JMP   DISP_EXIT

NOT_Rotate_L  CMPA  #Rotate_R
              BNE   NOT_Rotate_R
              JSR   Rotate_R_ST
              JMP   DISP_EXIT

NOT_Rotate_R  CMPA  #Rotate_180
              BNE   NOT_180
              JSR   Rotate_180_ST
              JMP   DISP_EXIT
              

NOT_180       CMPA #REVERSE
              BNE NOT_REVERSE
              JSR REVERSE_ST
              JMP DISP_EXIT

NOT_REVERSE   CMPA #Solve_180
              BNE NOT_Solve_180
              JSR Solve_180_ST
              JMP DISP_EXIT
              
NOT_Solve_180 CMPA #Solve_FWD
              BNE NOT_Solve_FWD
              JSR Solve_FWD_ST
              JMP DISP_EXIT             

NOT_Solve_FWD CMPA #Solve_Turn
              BNE NOT_Solve_Turn
              JSR Solve_Turn_ST
              JMP DISP_EXIT
                            
NOT_Solve_Turn CMPA #Solve_R
               BNE NOT_Solve_R
               JSR Solve_R_ST
               JMP DISP_EXIT              

NOT_Solve_R
DISP_EXIT     RTS


;States
*******************************************************************
FWD_ST            JSR CHECK_BIAS
                  CMPB #0
                  BNE FWD_RIGHT
                  BCLR PTT,%00100000 ; Turn off the right drive motor
                  BSET PTT,%00010000 ; Turn on the left drive motors
                  BRA FWD_JUNC_CHECK

FWD_RIGHT         BCLR PTT,%00010000 ; Turn off the left drive motor
                  BSET PTT,%00100000 ; Turn on the right drive motor

FWD_JUNC_CHECK    BRSET PORTAD0,$4,FWD_NO_BACKTRAC; If front bumper is pressed, do a 180
                  JSR INIT_Rotate_180
                  MOVB #Rotate_180,CRNT_STATE
                  BRA FWD_EXIT
    
FWD_NO_BACKTRAC   BRSET PORTAD0,$8,FWD_NO_SOLVE; If back bumper is pressed, begin solving
                  JSR INIT_Solve_180
                  MOVB #Solve_180,CRNT_STATE
                  BRA FWD_EXIT

FWD_NO_SOLVE      JSR CHECK_BACK                                                   
                  CMPB #0
                  BNE FWD_CHECK_LEFT
                  JSR INIT_REVERSE
                  MOVB #REVERSE,CRNT_STATE
                  
FWD_CHECK_LEFT    JSR CHECK_LEFT                                                   
                  CMPB #0
                  BEQ FWD_CHECK_RIGHT
                  JSR INIT_JUNC_DECIDER
                  MOVB #JUNC_DECIDER,CRNT_STATE
                  
FWD_CHECK_RIGHT   JSR CHECK_RIGHT                                                   
                  CMPB #0
                  BEQ FWD_EXIT
                  JSR INIT_JUNC_DECIDER
                  MOVB #JUNC_DECIDER,CRNT_STATE  
                  
FWD_EXIT          RTS ; return to the MAIN routine
*******************************************************************
JUNC_DECIDER_ST   JSR CHECK_FRONT
                  CMPB  #0
                  BNE   JUNC_FRONT
 
                  JSR CHECK_LEFT
                  CMPB  #0
                  BNE   JUNC_LEFT 

                  JSR INIT_Rotate_R           ;No front and left = l junction going right. rotate right state
                  JSR STORE_DIR
                  MOVB #Rotate_R,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT   
                    
JUNC_LEFT         JSR CHECK_RIGHT
                  CMPB #0
                  BNE JUNC_LR
                  
                  JSR INIT_Rotate_L           ;No front and right = l junction going left. rotate left state
                  JSR STORE_DIR
                  MOVB #Rotate_L,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT                      

JUNC_LR           JSR INIT_Rotate_L           ;T junction left and right. Go left
                  JSR STORE_DIR
                  MOVB #Rotate_L,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT
                  
JUNC_FRONT        JSR INIT_FWD                ;T junction with front. Change to fwd state
                  JSR STORE_DIR
                  MOVB #FWD,CRNT_STATE        

JUNC_DECIDER_EXIT RTS;
*******************************************************************
BACKTRACK_JUNC_ST LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
                  CMPA    T_180_ROT         ; the robot should go FWD
                  BNE     BACKTRACK_JUNC_EXIT       ; so;First, check if the car is done rotating. If not, don't check junctions

                  JSR CHECK_FRONT
                  CMPB  #0
                  BNE   BACKJUNC_FRONT
 
                  JSR CHECK_LEFT
                  CMPB  #0
                  BNE   BACKJUNC_LEFT 

                  JSR INIT_Rotate_R           ;No front and left = l junction going right. rotate right state
                  DEY
                  JSR STORE_DIR
                  MOVB #Rotate_R,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT   
                    
BACKJUNC_LEFT     JSR CHECK_RIGHT
                  CMPB #0
                  BNE BACKJUNC_LR
                  
                  JSR INIT_Rotate_L           ;No front and right = l junction going left. rotate left state
                  DEY
                  JSR STORE_DIR
                  MOVB #Rotate_L,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT               

BACKJUNC_LR       JSR INIT_Rotate_R           ;T junction left and right. This time, go right
                  DEY
                  JSR STORE_DIR
                  MOVB #Rotate_R,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT
                  
BACKJUNC_FRONT    JSR CHECK_LEFT
                  CMPB  #0 
                  BNE BACKJUNC_TLEFT
                                    
                  JSR INIT_Rotate_R           ;Front and no left = T junction going front/right. rotate right state
                  DEY
                  JSR STORE_DIR
                  MOVB #Rotate_R,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT

BACKJUNC_TLEFT    JSR INIT_Rotate_L           ;Front and Left= T junction going front/left. rotate left state
                  DEY
                  JSR STORE_DIR
                  MOVB #Rotate_L,CRNT_STATE
                  BRA JUNC_DECIDER_EXIT   

BACKTRACK_JUNC_EXIT   RTS ;
*******************************************************************
;After a rotation, what do we do?
;After a turn, set heading,go forward
Rotate_L_ST   LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
              CMPA    T_L_ROT         ; the robot should go FWD
              BNE     NO_FWD_RL       ; so
              
              JSR     INIT_FWD        ; initialize the FWD state
              MOVB    #FWD,CRNT_STATE ; set state to FWD
              BRA     Rotate_L_EXIT   ; and return
NO_FWD_RL     NOP ; Else              
Rotate_L_EXIT RTS; 
*******************************************************************
Rotate_R_ST   LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
              CMPA    T_R_ROT         ; the robot should go FWD
              BNE     NO_FWD_RR       ; so
              
              JSR     INIT_FWD        ; initialize the FWD state
              MOVB    #FWD,CRNT_STATE ; set state to FWD
              BRA     Rotate_R_EXIT   ; and return
              
NO_FWD_RR     NOP ; Else
Rotate_R_EXIT RTS; 
*******************************************************************
Rotate_180_ST LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
              CMPA    T_180_ROT         ; the robot should go FWD
              BNE     NO_FWD_180       ; so
              
              JSR     INIT_FWD        ; initialize the FWD state
              MOVB    #FWD,CRNT_STATE ; set state to FWD
              BRA     Rotate_180_EXIT   ; and return
              
NO_FWD_180     NOP ; Else
Rotate_180_EXIT      RTS;   
*******************************************************************
REVERSE_ST    JSR CHECK_BACK
              CMPA #0
              BNE REV_LINEHIT
              BRA REVERSE_EXIT

REV_LINEHIT   JSR INIT_FWD
              MOVB #FWD,CRNT_STATE
              BRA REVERSE_EXIT

REVERSE_EXIT  RTS
*******************************************************************
FWD_BACKTRACK_ST            JSR CHECK_BIAS
                            CMPB #0
                            BNE FWD_BACKTRACK_RIGHT
                            BCLR PTT,%00100000 ; Turn off the right drive motor
                            BSET PTT,%00010000 ; Turn on the left drive motors
                            BRA FWD_BACKTRACK_JUNC_CHECK

FWD_BACKTRACK_RIGHT         BCLR PTT,%00010000 ; Turn off the left drive motor
                            BSET PTT,%00100000 ; Turn on the right drive motor ;check for a junction, then call backtrack junc
    
FWD_BACKTRACK_JUNC_CHECK    JSR CHECK_BACK                                                   
                            CMPB #0
                            BNE FWD_BACKTRACK_CHECK_LEFT
                            JSR INIT_REVERSE
                            MOVB #REVERSE,CRNT_STATE
                  
FWD_BACKTRACK_CHECK_LEFT    JSR CHECK_LEFT                                                   
                            CMPB #0
                            BEQ FWD_BACKTRACK_CHECK_RIGHT
                            JSR INIT_BACKTRACK_JUNC
                            MOVB #BACKTRACK_JUNC,CRNT_STATE
                  
FWD_BACKTRACK_CHECK_RIGHT   JSR CHECK_RIGHT                                                   
                            CMPB #0
                            BEQ FWD_BACKTRACK_EXIT
                            JSR INIT_BACKTRACK_JUNC
                            MOVB #BACKTRACK_JUNC,CRNT_STATE  
                  
FWD_BACKTRACK_EXIT          RTS ; return to the MAIN routine
*******************************************************************
Solve_180_ST    LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
                CMPA    T_180_ROT         ; the robot should go FWD
                BNE     NO_FWD_Solve_180       ; so
                
                JSR     INIT_Solve_FWD        ; initialize the FWD state
                MOVB    #Solve_FWD,CRNT_STATE ; set state to FWD
                BRA     Solve_180_EXIT   ; and return
              
NO_FWD_Solve_180      NOP ; Else
Solve_180_EXIT       RTS;   
*******************************************************************
Solve_FWD_ST      JSR CHECK_BIAS
                  CMPB #0
                  BNE Solve_FWD_RIGHT
                  BCLR PTT,%00100000 ; Turn off the right drive motor
                  BSET PTT,%00010000 ; Turn on the left drive motors
                  BRA Solve_FWD_CHECK_LEFT      

Solve_FWD_RIGHT         BCLR PTT,%00010000 ; Turn off the left drive motor
                  BSET PTT,%00100000 ; Turn on the right drive motor


                  
Solve_FWD_CHECK_LEFT    JSR CHECK_LEFT                                                   
                  CMPB #0
                  LDAA JUNCNUM
                  DECA
                  STAA JUNCNUM
                  BEQ Solve_FWD_CHECK_RIGHT
                  JSR SOLVE
                  JSR INIT_Solve_Turn
                  MOVB #Solve_Turn,CRNT_STATE
                  
Solve_FWD_CHECK_RIGHT   JSR CHECK_RIGHT                                                   
                  CMPB #0
				  LDAA JUNCNUM
                  DECA
                  STAA JUNCNUM
                  BEQ Solve_FWD_EXIT
                  JSR SOLVE
                  JSR INIT_Solve_Turn
                  MOVB #Solve_Turn,CRNT_STATE  
                  
Solve_FWD_EXIT          RTS ; return to the MAIN routine
*******************************************************************
Solve_Turn_ST   LDAA TARGET_HEADING
                CMPA HEADING
                BNE  WRONG_HEADING
                
                JSR INIT_Solve_FWD
                MOVB #Solve_FWD,CRNT_STATE
                BRA Solve_Turn_EXIT

WRONG_HEADING   JSR INIT_Solve_R
                MOVB #Solve_R,CRNT_STATE

Solve_Turn_EXIT RTS ; Check if facing correct direction. If not, turn 90 degrees until we do
*******************************************************************
Solve_R_ST      LDAA    TOF_COUNTER     ; If Tc>Tfwdturn then
                CMPA    T_R_ROT         ; the robot should go FWD
                BNE     Solve_R_NOFWD       ; so
              
                JSR     INIT_Solve_Turn        ; initialize the FWD state
                MOVB    #Solve_Turn,CRNT_STATE ; set state to FWD
                NOP                     ; set the heading somehow ************
                BRA     Solve_R_EXIT   ; and return
              
Solve_R_NOFWD       NOP ; Else
Solve_R_EXIT   RTS; 

;State INITs
*******************************************************************
INIT_FWD        BCLR PORTA,%00000011 ; Set FWD direction for both motors
                BSET PTT,%00110000 ; Turn on the drive motors
                RTS:
*******************************************************************
INIT_JUNC_DECIDER     BCLR PTT,%00110000    ; Turn off the drive motors
                RTS; 
*******************************************************************
INIT_BACKTRACK_JUNC     JSR H_RIGHT          ; Rotate the heading right twice to set  
                        JSR H_RIGHT          ; the heading 180 degrees from where it was
                        BCLR PORTA,%00000001 ; Set FWD dir. for PORT (left) motor
                        BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                        BSET PTT,%00110000 ; Turn on the drive motors
                        LDAA TOF_COUNTER; Mark the turning time
                        ADDA #R_ROT_INT ; 
                        STAA T_180_ROT ; Store the time where left rot must stop
                        RTS;
*******************************************************************
INIT_Rotate_L   JSR H_LEFT
                BCLR PORTA,%00000010 ; Set FWD dir. for STARBOARD (right)
                BSET PORTA,%00000001 ; Set REV dir. for PORT (left)
                BSET PTT,%00110000 ; Turn on the drive motors
                LDAA TOF_COUNTER; Mark the turning time
                ADDA #L_ROT_INT ; 
                STAA T_L_ROT ; Store the time where left rot must stop
                RTS;
*******************************************************************
INIT_Rotate_R   JSR H_RIGHT
                BCLR PORTA,%00000001 ; Set FWD dir. for PORT (left) motor
                BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                BSET PTT,%00110000 ; Turn on the drive motors
                LDAA TOF_COUNTER; Mark the turning time
                ADDA #R_ROT_INT ; 
                STAA T_R_ROT ; Store the time where left rot must stop
                RTS;
*******************************************************************
INIT_Rotate_180 JSR H_RIGHT
                JSR H_RIGHT
                BCLR PORTA,%00000001 ; Set FWD dir. for PORT (left) motor
                BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                BSET PTT,%00110000 ; Turn on the drive motors
                LDAA TOF_COUNTER; Mark the turning time
                ADDA #R_ROT_INT ; 
                STAA T_180_ROT ; Store the time where left rot must stop
                RTS;
*******************************************************************
INIT_REVERSE    BSET PORTA,%00000001 ; Set REV dir. for PORT (left)
                BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                BSET PTT,%00110000 ; Turn on the drive motors
                RTS;
*******************************************************************
INIT_Solve_180  DEY
                BCLR PORTA,%00000001 ; Set FWD dir. for PORT (left) motor
                BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                BSET PTT,%00110000 ; Turn on the drive motors
                LDAA TOF_COUNTER; Mark the turning time
                ADDA #R_ROT_INT ; 
                STAA T_180_ROT ; Store the time where left rot must stop
                RTS;
*******************************************************************
INIT_Solve_FWD  BCLR PORTA,%00000011 ; Set FWD direction for both motors
                BSET PTT,%00110000 ; Turn on the drive motors
               
                RTS;
*******************************************************************
INIT_Solve_Turn BCLR PTT,%00110000  ;Shut of the motors while the car is deciding to turn or not
                RTS;
*******************************************************************                
INIT_Solve_R    JSR H_RIGHT
                BCLR PORTA,%00000001 ; Set FWD dir. for PORT (left) motor
                BSET PORTA,%00000010 ; Set REV dir. for STARBOARD (right)
                BSET PTT,%00110000 ; Turn on the drive motors
                LDAA TOF_COUNTER; Mark the turning time
                ADDA #R_ROT_INT ; 
                STAA T_R_ROT ; Store the time where left rot must stop
                RTS; 
                
; Custom subroutines
*******************************************************************
*******************************************************************
; Check Pattern subroutines
CHECK_FRONT       LDAB    #0
                  LDAA    SENSOR_BOW
                  CMPA    THRESHOLD
                  BPL     NO_FRONT       
              
                  LDAB    #1             ;Here is there is a front detected                     

NO_FRONT          NOP                   ;Do nothing if no front detected          
CHECK_FRONT_EXIT  RTS
*******************************************************************
CHECK_BACK        LDAB    #0
                  LDAA    SENSOR_MID
                  CMPA    THRESHOLD
                  BPL     NO_BACK       
              
                  LDAB    #1             ;Here is there is a front detected                     

NO_BACK           NOP                   ;Do nothing if no front detected          
CHECK_BACK_EXIT   RTS
*******************************************************************
CHECK_LEFT        LDAB    #0
                  LDAA    SENSOR_PORT
                  CMPA    THRESHOLD
                  BPL     NO_LEFT       
              
                  LDAB    #1             ;Here is there is a front detected                     

NO_LEFT           NOP                   ;Do nothing if no front detected          
CHECK_LEFT_EXIT   RTS
*******************************************************************
CHECK_RIGHT       LDAB    #0
                  LDAA    SENSOR_STBD
                  CMPA    THRESHOLD
                  BPL     NO_RIGHT       
              
                  LDAB    #1             ;Here is there is a front detected                     

NO_RIGHT           NOP                   ;Do nothing if no front detected          
CHECK_RIGHT_EXIT   RTS
*******************************************************************
CHECK_BIAS        LDAB #0
                  LDAA SENSOR_LINE
                  CMPA LINETHRESHOLD
                  BPL BIAS_RIGHT
                  LDAB #1
BIAS_RIGHT        RTS
*******************************************************************
SOLVE        LDY #JUNC1               
              LDAA JUNCNUM               
SOLVE_SETY    BEQ SOLVE_GETTAR              
              INY
              DECA 
              BRA SOLVE_SETY   
SOLVE_GETTAR LDAA 0,Y
              DEY
              CMPA #2
              BMI SOLVE_NE              
              DECA 
              DECA
              STAA TARGET_HEADING
              RTS  
SOLVE_NE     INCA
              INCA
              STAA TARGET_HEADING
              RTS                

*******************************************************************
STORE_DIR     LDY #JUNC1               
              LDAA JUNCNUM               
STOREDIR_SETY BEQ STOREDIR_STR
              INY
              DECA 
              BRA STOREDIR_SETY  
STOREDIR_STR  LDAA HEADING
              STAA 0,Y
              LDAB JUNCNUM
              INCB
              STAB JUNCNUM
              INY
              RTS

*******************************************************************
H_LEFT        LDAB HEADING
              CMPB #0
              BNE H_LEFT_NOT0
        
              LDAB #3
              STAB HEADING
              RTS

H_LEFT_NOT0   DECB
              STAB HEADING
              RTS 
*******************************************************************
H_RIGHT       LDAB HEADING
              CMPB #3
              BNE H_RIGHT_NOT3
        
              LDAB #0
              STAB HEADING
              RTS

H_RIGHT_NOT3  INCB
              STAB HEADING
              RTS 

; Guider.asm subroutines
*******************************************************************
*******************************************************************

;---------------------------------------------------------------------------
; Initialize ports

INIT            BCLR DDRAD,$FF      ; Make PORTAD an input (DDRAD @ $0272)
                BSET DDRA,$FF       ; Make PORTA an output (DDRA @ $0002)
                BSET DDRB,$FF       ; Make PORTB an output (DDRB @ $0003)
                BSET DDRJ,$C0       ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                RTS
                
;---------------------------------------------------------------------------
; Initialize the ADC
openADC         MOVB #$80,ATDCTL2   ; Turn on ADC (ATDCTL2 @ $0082)
                LDY #1              ; Wait for 50 us for ADC to be ready
                JSR del_50us        ; - " -
                MOVB #$20,ATDCTL3   ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                MOVB #$97,ATDCTL4   ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                RTS
                

;---------------------------------------------------------------------------
; Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
; correspond to the ’illuminated’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_ON       BSET PORTA,%00100000 ; Set bit 5
                RTS
;
; Guider LEDs OFF

; This routine disables the guider LEDs. Readings of the sensor
; correspond to the ’ambient lighting’ situation.

; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed

G_LEDS_OFF      BCLR PORTA,%00100000 ; Clear bit 5
                RTS
;---------------------------------------------------------------------------
; Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
; registers.
; Note: Do not confuse the analog multiplexer on the Guider board with the
; multiplexer in the HCS12. The guider board mux must be set to the
; appropriate channel using the SELECT_SENSOR routine. The HCS12 always
; reads the selected sensor on the HCS12 A/D channel AN1.
; The A/D conversion mode used in this routine is to read the A/D channel
; AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
; used in this routine is the value from AN1, read from ATDDR0. However,
; other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
; Passed: None
; Returns: Sensor readings in:
; SENSOR_LINE (0) (Sensor E/F)
; SENSOR_BOW (1) (Sensor A)
; SENSOR_PORT (2) (Sensor B)
; SENSOR_MID (3) (Sensor C)
; SENSOR_STBD (4) (Sensor D)
; Note:
; The sensor number is shown in brackets
;
; Algorithm:
; Initialize the sensor number to 0
; Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
; Repeat
; Read ATDSTAT0
; Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
; Store the contents of ATDDR0L at the pointer
; If the pointer is at the last entry in Sensor Array, then
; Exit
; Else
; Increment the sensor number
; Increment the pointer
; Loop again.
READ_SENSORS    CLR SENSOR_NUM        ; Select sensor number 0
                LDX #SENSOR_LINE      ; Point at the start of the sensor array
    
RS_MAIN_LOOP    LDAA SENSOR_NUM       ; Select the correct sensor input
                JSR SELECT_SENSOR     ; on the hardware
                LDY #400 ;             20 ms delay to allow the
                JSR del_50us          ; sensor to stabilize
                
                LDAA #%10000001       ; Start A/D conversion on AN1
                STAA ATDCTL5
                BRCLR ATDSTAT0,$80,*  ; Repeat until A/D signals done
                
                LDAA ATDDR0L          ; A/D conversion is complete in ATDDR0L
                STAA 0,X              ; so copy it to the sensor register
                CPX #SENSOR_STBD      ; If this is the last reading
                BEQ RS_EXIT           ; Then exit
                
                INC SENSOR_NUM        ; Else, increment the sensor number
                INX                   ; and the pointer into the sensor array
                BRA RS_MAIN_LOOP      ; and do it again

RS_EXIT         RTS
;---------------------------------------------------------------------------
; Select Sensor
; This routine selects the sensor number passed in ACCA. The motor direction
; bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
; same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
; which selects the guider sensor to be connected to AN1.
; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed
; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
; the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
; 11100011. The zeros in the mask clear the corresponding bits in the
; TEMP. The 1’s have no effect.
; Next, move the sensor selection number left two positions to align it
; with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
; the mask 00011100. The zeros in the mask clear everything except
; the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
; The effect is that only bits 2,3,4 are changed in the TEMP, and these
; bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.
SELECT_SENSOR   PSHA                  ; Save the sensor number for the moment

                LDAA PORTA            ; Clear the sensor selection bits to zeros
                ANDA #%11100011       ;
                STAA TEMP             ; and save it into TEMP
                
                PULA                  ; Get the sensor number
                ASLA                  ; Shift the selection number left, twice
                ASLA                  ;
                ANDA #%00011100       ; Clear irrelevant bit positions
                
                ORAA TEMP             ; OR it into the sensor bit positions
                STAA PORTA            ; Update the hardware
                RTS

                
 
;---------------------------------------------------------------------------
; 50 Microsecond Delay
del_50us        PSHX                    ; (2 E-clk) Protect the X register
eloop           LDX #300                ; (2 E-clk) Initialize the inner loop counter
iloop           NOP                     ; (1 E-clk) No operation
                DBNE X,iloop            ; (3 E-clk) If the inner cntr not 0, loop again
                DBNE Y,eloop            ; (3 E-clk) If the outer cntr not 0, loop again
                PULX                    ; (3 E-clk) Restore the X register
                RTS                     ; (5 E-clk) Else return

;TOF
*******************************************************************
ENABLE_TOF    LDAA  #%10000000
              STAA  TSCR1         ; Enable TCNT
              STAA  TFLG2         ; Clear TOF
              LDAA  #%10000100    ; Enable TOI and select prescale factor equal to 16
              STAA  TSCR2
              RTS
*******************************************************************
TOF_ISR       INC   TOF_COUNTER
              LDAA  #%10000000    ; Clear
              STAA  TFLG2         ; TOF
              RTI

*******************************************************************
* Interrupt Vectors *
*******************************************************************
  ORG $FFFE
  DC.W Entry ; Reset Vector
  ORG $FFDE
  DC.W TOF_ISR ; Timer Overflow Interrupt Vector
