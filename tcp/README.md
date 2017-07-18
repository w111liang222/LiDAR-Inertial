# Interface for TCP


## Requirements
* C++ compiler (support C++11)
* boost library (version 1.61)

## Usage examples

#### TCP Server（包含TCPServer.h）

* 创建TCP Server实例，设置监听端口port
```
 TCPServer myServer(port);
```

* 等待Client连接
```
 myServer.waitForConnect();
```

* 发送数据到Client
```
 myServer.TCPServerSend(buf,len);
```

* 从Client接收数据
```
 myServer.TCPServerReceive(buf,len);
```
 
#### TCP Client（包含TCPClient.h）

* 创建TCP Client实例，连接服务器（xxx.xxx.xxx.xxx），端口port，并等待连接完成
```
 TCPClient myClient("xxx.xxx.xxx.xxx", port);
```

* 发送数据到Server
```
 myClient.TCPClientSend(buf,len);
```

* 从Server接收数据
```
 myClient.TCPClientReceive(buf,len);
```

## Help and Support
contact: 15lwang@tongji.edu.cn
