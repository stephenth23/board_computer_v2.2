# board_computer_v2.3

BC v2.3

States:
  1. BOOTING UP
    - LED on
    - a 3-tone buzz from the speaker
  2. WAITING FOR COMMAND
    - LED off
    - no soud
  3. ARMED AND RUNNING
    - press button for arming the PC
    - alarm sound buzzer
    - small flash + bip sound every 2 seconds
  4. ANY ERROR
    - every procces will stop
    - the BC will flash the LED
    
Contents:
  - accelometer: measure g-forces $ the direction of the rocket
  - barometter: claculates altitude
  - SD card module: write data to a SD card
