# New kit for Lidar M1C1 Mini
"Driver" and Graphical interface for a chinese cheap lidar, currently for Linux/Unix/WSL


This program utilizes a *C*-coded serial reader and *pygame*-based graphical view of the lidar output. The lidar used was purchased on AliExpress (https://www.aliexpress.com/snapshot/0.html?spm=a2g0o.9042647.6.2.559e37a1jYP0hF&orderId=8133980903635463&productId=4000251359842) and has an 8m range which is pretty good. 

This is my own development and it was based on a chinese datasheet which I translated to Brazilian Portuguese using Google Translate, since the english version on AliExpress site was incomplete. 
The main points in this code is to use C for a very fast serial reading and pipes to connect to a pygame based interface, which is simple to modify and understand yet beautiful.

The pipe connection was developed using ChatGPT 4o. It felt like cheating but saved a lot of time coding so, if you are getting too old to dedicate a lot (and I mean a lot) of time debugging, give it a try! You still have to give guidelines and think about the logic of the code but the brick-laying work is done by your digital intern!

I will also be publishing the SOCKETs version (both Unix-socket and TCP-Socket) that is even faster and prettier!
