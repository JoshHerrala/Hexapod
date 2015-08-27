# Hexapod
This project is the code for an Arduino based hexapod using an Adafruit servo driver and SRF04 range finder. The code is intentionally very crude so that younger/less skilled users can understand what is going on. There are lots of comments. 

Currently the code uses the range finder to spot if an object is within 10cm, and stops until the object is moved. The robot waves its front legs until the object is moved. When time permits, I will add code to drive a servo with the range finder perched on top so it can sweep the area and look for open routes. 
