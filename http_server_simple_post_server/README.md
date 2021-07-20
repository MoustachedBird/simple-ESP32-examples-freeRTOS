## Usage

* Change the SSID and PASSWORD of your WIFI in the main folder, file_server.c 

* In order to test the file server:
    1. compile and burn the firmware `idf.py -p PORT flash`
    2. flash the IMG file using: 
	python spiffsgen.py 1048576 /home/moustachedbird/esp/esp32_projects/http_server5/spiffs_files /home/moustachedbird/esp/esp32_projects/http_server5/spiffs_image.img

    3. test the example:
	*Open one terminal and excecute `idf.py -p PORT monitor`

    4. Send POST REQUESTS to the server: 
	*PYTHON OPTION: open another terminal and run this command:  python client.py <ip> <port> <msg> 
		example: python client.py 192.168.0.101 80 hello

	*INDEX.HTML OPTION: open your web navigator and write your IP example: 192.168.0.101
	you will see the index.html file if everything is ok. Click the button and you will send json content to the server 
 

