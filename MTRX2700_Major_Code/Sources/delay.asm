
; export symbols                                                       
            XDEF  delay1ms, delay10ms, delay50us, delay1s


;**************************************************************
; 
; Delay Function
;           
; This function generates an 1 ms delay.
;     (1 clock cycle = 41.66 ns
;     (4 clock cycles x 6000 loops x 41.66 ns = 1 ms) 
;
;**************************************************************
            
delay1ms:
            LDX   #6000     ; Calculated no. of loops needed 
                        
loop1ms:  
            DEX             ; Decrement X by 1 (1 clock cycle)
            BNE   loop1ms   ; Loop again if not equal to 0 (3 clock cycles) 
            
            RTS             ; Return from subroutine
            
  
           
delay10ms:
            LDX   #60000    ; Calculated no. of loops needed 
                        
loop10ms:  
            DEX             ; Decrement X by 1 (1 clock cycle)
            BNE   loop10ms  ; Loop again if not equal to 0 (3 clock cycles) 
            
            RTS             ; Return from subroutine 
            
            

delay50us:
            LDX   #300      ; Calculated no. of loops needed 
                        
loop50us:  
            DEX             ; Decrement X by 1 (1 clock cycle)
            BNE   loop50us  ; Loop again if not equal to 0 (3 clock cycles) 
            
            RTS             ; Return from subroutine   
            
            
            

;**************************************************************
;
; Delay Function
;
; This function generates a delay by using a certain number of
; clock cycles. The length of the delay is ; dependent on the
; values for C1 and C2.
;
; The number of clock cycles is given by the formula:
; 
;     TC = 5*C1*C2 + 5*C2
;
;
; Example:
; To achieve a  1 second delay (24,000,000 clock cycles),
; the values C1 = 4799 and C2 = 1000 can be used.
; This evaluates to:
;
;     TC = 5*4799*1000 + 5*1000 = 24,000,000 clock cycles
; 
;**************************************************************

delay1s:
            LDX   #4799                     ; Initialise the counter for the first loop (nested loop)
            LDY   #1000                     ; Initialise the counter for the second loop (outer loop)
      
loop1:            
            DEX                             ; Decrement X
            
            BEQ   loop2                     ; If X is 0, branch to loop2
            BRA   loop1                     ; Otherwise, branch to loop1
                        
loop2:                  
            LDX   #4799                     ; Reset the value of X to C1
                        
            DEY                             ; Decrement Y
            
            BNE   loop1                     ; If Y is not 0, branch to loop1 
            RTS                             ; Otherwise, return from subroutine                  
            
 
                        
          