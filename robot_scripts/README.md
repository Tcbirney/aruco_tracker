To run these scripts, you will need a DFRobot Baron platform with a Raspberry Pi 4 running ubuntu 20.*.

Make sure your computer and the Pi have OpenCV, ROS, numpy, and socket isntalled.

First, run `pi_receiver.py` on the Pi through an ssh connection. This will open a listener port on the Pi. Then run `computer_stream_pipeline.py` on your computer. This will setup a listener to the Pi's image stream and connect a sender to the Pi's instructions listener. Finally, run `pi_stream.py` on your Pi through ssh. This will start sending the image data to the computer. The Pi should now move.

To perform onboard processing, run `pi_stream_w_proc.py` on your Pi through ssh instead. This will be slower but does not necessitate a computer be connected to the Pi, or that the Pi be connected to Wi-Fi at all.

If any problems arise, please email nnovak@umd.edu, the maintainer of this code.