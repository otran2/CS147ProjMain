CS 147 Audio Motion Occupancy Tracker

Before running our code, you first need to have a working server, and you would add it's server address to the variable kHostname in main.cpp.
Additionally, you need to decide what path you want to provide it, for us our server just used /path.

An important thing to note is that before running our program, we need init_NVS() in our setup at least once before we run the main code.
You can uncomment it on line 49, this step is essentially needed to flash the necessary information onto our ESP 32 board's memory.
