# raspberry_pi_dro
Raspberry PI DRO for the Lathe, with Frequency drive control and soft stops

If you ever thought a manual lathe could be more intelligent, and a CNC lathe takes too long to program for one of jobs, this project might be for you. Thread cutting, the cut cycle is slow, the backout cycle is just as painfully slow. It wwoul dbe nice to have a seperate speed for cut and backout cycles.

Features:

1) Simple button controls for forward / Reverse and Stop
2) Set speed on forward and reverse cycles.
3) Soft stops
4) automatic tapping function with depth and reverse out
5) Direct Glass scale interface

Controls:

[ Forward ]        [ Clear ]
[ Reverse ] [Stop] [ Enter ] [ Rotary Enc ]

Functions:
  Fwd <Ent>       " Run forward
  Fwd <Rot> < Ent> " forward with speed adjust
  Rev <Ent>        " Reverse
  Rev <Rot> <Ent>  " Reverse with speed adjust
  Stop             " Stop with break
  
  Stop <Ent>     " Set stop position based  on current Z encoder position, only active when spindle not running
  Stop <Rot> <Ent> " Set stop position with encoder based on current position, only active when spindle is not running
  
  Emergency Stop  " Stop 

Hardware:
The GPIO speed tests I have seen seem fast enough, the problem is if the cpu is busy some steps might get lost. I ahve been wanting to make use of a few HCTL2000 IC's in the parts bin. 

GPIO 3.3V to 5V
          5,6,13,16,19,20,21,26 <- D0-D7
          4  -> CLK
          23  -> SEL
          24  -> EN
          
          
abyz.co.uk/rpi/pigpio/code/minimal_clk.zip

https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino
https://www.tansi.org/rp/interfacing5v.html
https://github.com/machinekit
https://github.com/hzeller/rpi-gpio-dma-demo
https://gist.github.com/ast/a19813fce9d34c7240091db11b8190dd

