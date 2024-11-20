/*
 * @LastEditors: qingmeijiupiao
 * @Description: 重庆邮电大学HXC战队ESP-NOW二次封装库,指定了发包格式
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-20 11:15:38
 */

#ifndef esp_now_hpp
#define esp_now_hpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <map>




//发送数据包失败最大重试次数
#define MAX_RETRY 5

//数据包密钥
constexpr uint16_t secret_key=0xFEFE;

//广播地址
uint8_t broadcastMacAddress[] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//记录是否收到数据包，用于判断是否连接
bool is_conect = false;








/*↓↓↓↓声明↓↓↓↓*/

//数据包
struct HXC_ESPNOW_data_pakage;

//回调函数
using callback_func =std::function<void(HXC_ESPNOW_data_pakage)>;

//ESP-NOW初始化
void esp_now_setup(uint8_t* receive_MAC=broadcastMacAddress);

//发送数据
esp_err_t esp_now_send_package(String name,uint8_t* data,int datalen,uint8_t* receive_MAC=broadcastMacAddress);

//添加回调函数
void add_esp_now_callback(String package_name,callback_func func);

//移除回调函数
void remove_esp_now_callback(String package_name);












/*↓↓↓↓定义↓↓↓↓*/

//数据包格式
struct HXC_ESPNOW_data_pakage {
  uint16_t header_code=secret_key;//数据包头,作为密钥使用
  uint8_t name_len;
  uint8_t data_len;
  String package_name;
  uint8_t data[256];
  //添加名字
  void add_name(String _name){
    package_name=_name;
    name_len=package_name.length();
  }
  //添加数据
  void add_data(uint8_t* _data,int _datalen){
    data_len=_datalen;
    for(int i=0;i<_datalen;i++){
      data[i]=_data[i];
    }
  }
  //获取数据
  void get_data(uint8_t* _data){
    _data[0]=header_code/256;
    _data[1]=header_code%256;
    _data[2]=package_name.length();
    _data[3]=data_len;
    memcpy(_data+4,package_name.c_str(),package_name.length());
    memcpy(_data+4+package_name.length(),data,data_len);
  }

  //解码数组到结构体对象
  void decode(uint8_t* _data,int _datalen){
    header_code=_data[0]+_data[1]*256;
    name_len=_data[2];
    data_len=_data[3];
    package_name="";
    for(int i=0;i<name_len;i++){
      package_name+=char(_data[4+i]);
    }
    for(int i=0;i<data_len;i++){
      data[i]=_data[4+name_len+i];
    }
  }
  //获取数据包长度
  int get_len(){
    return 4+name_len+data_len;
  }

};


static std::map<String, callback_func> callback_map;//回调函数map

void add_esp_now_callback(String package_name,callback_func func){
  callback_map[package_name]=func;
}
void remove_esp_now_callback(String package_name){
  callback_map.erase(package_name);
};

static HXC_ESPNOW_data_pakage re_data;//数据包缓存对象

//接收数据时的回调函数，收到数据时自动运行
void OnESPNOWDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  //检查是否是数据包
  if(len<2) return;
  if(*(uint16_t*)data!=secret_key) return;
  re_data.decode((uint8_t*)data,len);
    //检查是否是需要运行回调函数的数据包
  if(callback_map.count( re_data.package_name )!=0){
    callback_map[re_data.package_name](re_data);
  }
  is_conect=true;
}


static esp_now_peer_info_t peerInfo;

//ESP-NOW初始化
void esp_now_setup(uint8_t* receive_MAC){

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW initialization failed");
      return;
  }

  peerInfo.ifidx = WIFI_IF_STA;
  memcpy(peerInfo.peer_addr, receive_MAC, 6);
  esp_now_add_peer(&peerInfo);

  if(receive_MAC!=broadcastMacAddress){
    memcpy(peerInfo.peer_addr, broadcastMacAddress, 6);
    esp_now_add_peer(&peerInfo);
  }
  esp_now_register_recv_cb(OnESPNOWDataRecv);
} 


//通过espnow发送数据包
esp_err_t esp_now_send_package(String name,uint8_t* data,int datalen,uint8_t* receive_MAC){
  HXC_ESPNOW_data_pakage send_data;
  send_data.add_name(name);
  send_data.add_data(data,datalen);
  uint8_t send_data_array[send_data.get_len()];
  send_data.get_data(send_data_array);
  //发送
  for(int i=0;i<MAX_RETRY;i++){
    auto err = esp_now_send(receive_MAC,send_data_array,send_data.get_len());
    if (err == ESP_OK)  return ESP_OK;
    delay(20);
  }
  return ESP_FAIL;
}

#endif
/*
                                              .=%@#=.                                               
                                            -*@@@@@@@#=.                                            
                                         .+%@@@@@@@@@@@@#=                                          
                                       -#@@@@@@@* =@@@@@@@@*:                                       
                                     =%@@@@@@@@=   -@@@@@@@@@#-                                     
                                  .+@@@@@@@@@@-     .@@@@@@@@@@%=                                   
                                .+@@@@@@@@@@@@-     +@@@@@@@@@@@@@+.                                
                               +@@@@@@@@@@@@@@@    .@@@@@@@@@@@@@@@@+.                              
                             =@@@@@@@@@@@@@@@%-     =%@@%@@@@@@@@@@@@@=                             
                           -%@@@@@@@@@@@@+..     .       -@@@@@@@@@@@@@%-                           
                         .#@@@@@@@@@@@@@#       -@+       +@@@@@@@@@@@@@@#:                         
                        +@@@@@@@@@@@@@@@@+     +@@@+     =@@@@@@@@@@@@@@@@@+                        
                      :%@@@@@@@@@@@@@@@@@+    *@@@@*     =@@@@@@@@@@@@@@@@@@%-                      
                     +@@@@@@@@@@@@@@#+*+-   .#@@@@+       :+*+*@@@@@@@@@@@@@@@*                     
                   :%@@@@@@@@@@@@@@+       :%@@@@-    .-       -@@@@@@@@@@@@@@@%:                   
                  =@@@@@@@@@@@@@@@@-      -@@@@%:    .%@+      =@@@@@@@@@@@@@@@@@=                  
                 *@@@@@@@@@@@@@@@@@@.    =@@@@#.    -@@@@+    =@@@@@@@@@@@@@@@@@@@#                 
               .%@@@@@@@@@@@@@@@@@@+    +@@@@*     =@@@@%:    .#@@@@@@@@@@@@@@@@@@@%.               
              :@@@@@@@@@@@@@@@%:::.    #@@@@+     +@@@@#        .::.*@@@@@@@@@@@@@@@@-              
             -@@@@@@@@@@@@@@@%       .%@@@@=     *@@@@*     +-       *@@@@@@@@@@@@@@@@=             
            =@@@@@@@@@@@@@@@@@#.    -@@@@@-    :%@@@@=    .#@@+     +@@@@@@@@@@@@@@@@@@=            
           =@@@@@@@@@@@@@@@@@@@:    =====.     -+===:     :====     @@@@@@@@@@@@@@@@@@@@+           
          +@@@@@@@@@@@@@@@#%%#-                                     :*%%#%@@@@@@@@@@@@@@@+          
         =@@@@@@@@@@@@@@%.       ...........................              *@@@@@@@@@@@@@@@=         
        =@@@@@@@@@@@@@@@+      .#@@@@@@@@@@@@@@@@@@@@@@@@@@#     .*:      =@@@@@@@@@@@@@@@@-        
       -@@@@@@@@@@@@@@@@@=    .%@@@@@@@@@@@@@@@@@@@@@@@@@@#     :@@@-    =@@@@@@@@@@@@@@@@@@:       
      :@@@@@@@@@@@@@@@@@%.   -@@@@%+=====================:     -@@@@%    :%@@@@@@@@@@@@@@@@@@.      
      %@@@@@@@@@@@@@=-=:    =@@@@#.                           +@@@@#.      -=--%@@@@@@@@@@@@@%      
     #@@@@@@@@@@@@@:       +@@@@*      ............. .       *@@@@*             %@@@@@@@@@@@@@+     
    =@@@@@@@@@@@@@@#.     #@@@@+     +@@@@@@@@@@@@@@@#.    .#@@@@+     +#.     +@@@@@@@@@@@@@@@:    
   .@@@@@@@@@@@@@@@@-   .%@@@@=     *@@@@@@@@@@@@@@@#     :%@@@@-     *@@%:    @@@@@@@@@@@@@@@@%    
   %@@@@@@@@@@@%%%#=   :@@@@@:    .#@@@@+-----------     -@@@@@:     #@@@@=    :#%%%@@@@@@@@@@@@*   
  =@@@@@@@@@@@=       -@@@@%.    :%@@@@-                =@@@@%.    .%@@@@=          :%@@@@@@@@@@@:  
  @@@@@@@@@@@%.      =@@@@#     -@@@@%:    .:::-:      +@@@@#     :@@@@@:    .       +@@@@@@@@@@@#  
 +@@@@@@@@@@@@@.    *@@@@*     =@@@@#.    -@@@@@:     #@@@@+     =@@@@%.    -@#     +@@@@@@@@@@@@@- 
.@@@@@@@@@@@@@#    *@%@%=     +@@@@*     =@@@@#.    .#@@@%=     +@@@@#     =@@@%.   =@@@@@@@@@@@@@% 
+@@@@@@@@*-==-                .          .           . ..       .....      .....     .=+=+@@@@@@@@@-
%@@@@@@@+                                                                                 -@@@@@@@@#
@@@@@@@-       =#%#=     -#%%#-     -#%%*.     +%%%*.    .*%%#=     :#%%#-     =%%%*.      .#@@@@@@@
@@@@@@=.::::::*@@@@@*:::-@@@@@@-:::=@@@@@%::::*@@@@@#::::%@@@@@+:---@@@@@@=---+@@@@@%------:=@@@@@@@
=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+
 *@@@@@@@@@@@@@@@@@@@@@@@@@@@%%##**++===----:::::------===++***##%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@* 
  -#@@@@@@@@@@@@@@@@%#*+=-:.                                        ..::-=+*##%@@@@@@@@@@@@@@@@@#-  
    :=*%@@@@@%#*=-:                                                             .:-=+*#%%%%##+-.    
                                                                                        
        K####      #####     ###    ###  ######.   ##########     K##    ### ###    ##W    ####W    
       #######    #######    ###    ###  ########  ##########     ###    ### ###   ###   W######    
      W###G####  ###W ####   ###    ###  ######### ##########     ###    ###  ##   ###   ###W####   
      ###   ###  ###   ###   ###    ##  ###    ###    ###         ###    ###  ### t##   ###   ###   
     G##    #   ###    ###   ##     ##  ###    ###    ###         ###    ###  ### ###   ##W         
     ###        ###    ###   ##    ###  ###    ###    ###         ##L    ##   ### ##   ###          
     ###        ###    ###  K##    ###  ###    ###    ###         ##     ##    #####   ###          
     ###       ,##     ###  ###    ###  ###   ###,    ##         G##    ###    ####    ###          
    W##        ###     ###  ###    ###  #########     ##         ##########    ####    ###          
    ###        ###     ###  ###    ###  ########     ###         ##########    ###i   K##           
    ###        ###     ###  ###    ##  #######       ###         ###    ###    ####   ###           
    ###        ###     ###  ##     ##  ###           ###         ###    ###   ##W##   ###           
    ###        ###     ##i  ##    ###  ###           ###         ###    ##    ## ##   ###           
    ###        ###    ###  ,##    ###  ###           ###         ##     ##   ### ##   ###           
    ###    ### ###    ###  K##    ###  ###           ##         t##    ###   ##  ###  ###    ###    
    ###   G##i ###   ###   .##   ###.  ##t           ##         ###    ###  ###  ###  W##,   ###    
     ########  W##W#####    ########   ##           ###         ###    ###  ##    ##   ####W###     
     #######    #######     #######   ###           ###         ###    ### ###    ##.  #######      
      #####      #####       #####    ###           ###         ###    ### ##W    ###   #####       
                   ###                                                                              
                   ###                                                                              
                   #####                                                                            
                    ####                                                                            
                      K                                                                             
*/