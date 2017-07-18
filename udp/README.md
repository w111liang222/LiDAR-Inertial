# Interface for UDP


## Requirements
* C++ compiler (support C++11)
* boost library (version 1.61)

## Usage examples

#### UDP Server（包含UDPServer.h）

* 创建UDP Server实例，设置监听端口port
```
 UDPServer myUDPServer(port);
```

* 从Client接收数据
```
 myUDPServer.UDPServerReceive(buf,len);
```
 
#### UDP Client（包含UDPClient.h）

* 创建UDP Client实例，发送端口port
```
 UDPClient myUDPClient(port);
```

* 发送数据到Server
```
 myUDPClient.UDPClientSendto(buf,len);
```

## Help and Support
contact: 15lwang@tongji.edu.cn
