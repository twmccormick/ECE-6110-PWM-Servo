# ECE 6110 PWM Servo 
 
 ECE 6110 - Tyler McCormick - Extentsion 

 This code simply creates a PWM signal with a period of 20ms and simply changes the duty cycle from 5% to 10% back and forth with a small delay in bettwen each change.
 
 This generated PWM signal is then attached to one of the pins of the development board, which is then connected to the signal wire of a small servo.
 
 After generating the PWM signal and connecting it to an external pin on the board, a small amount of code simply moves the 
 servo about 90° back and forth by altering the CCR1 value of the timer, which changes the duty cycle of the PWM signal from
 5% to 10%.