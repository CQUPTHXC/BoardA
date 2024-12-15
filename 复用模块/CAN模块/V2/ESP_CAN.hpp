/*
 * @version: 2.0
 * @LastEditors: qingmeijiupiao
 * @Description: 对twai库的二次封装，API相比1.0大幅修改，不兼容
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-16 01:48:38
 */

#ifndef ESP_CAN_HPP
#define ESP_CAN_HPP
#include "driver/twai.h" //can驱动,esp32sdk自带
#include <map>

/*↓↓↓本文件的声明↓↓↓*/

//CAN基类,需要用到CAN时应该使用此类作为传入类型，方便扩展不同的CAN方式
class HXC_CAN;

//TWAI封装类
class HXC_TWAI;

//CAN消息结构体
struct HXC_CAN_message_t;


//CAN速率枚举
enum CAN_RATE{
  CAN_RATE_1MBIT,
  CAN_RATE_800KBIT,
  CAN_RATE_500KBIT,
  CAN_RATE_250KBIT,
  CAN_RATE_125KBIT,
  CAN_RATE_100KBIT
};


/*↑↑↑本文件的声明↑↑↑*/





//CAN消息接收回调函数
using HXC_can_feedback_func= std::function<void(HXC_CAN_message_t* can_message)>;

struct HXC_CAN_message_t{
  bool extd=0;           /**< 扩展帧格式标志（29位ID） */
  bool rtr=0;            /**< 远程帧标志 */
  bool self=0;           /**< 自我接收请求。接收时无效。 */
  uint32_t identifier;                /**< 11或29位标识符 */
  uint8_t data_length_code;           /**< 数据长度代码 */
  uint8_t data[8];    /**< 数据字节（在RTR帧中无关） */
};




class HXC_CAN{
  public:

  //防止值传递CAN对象
  //删除拷贝构造函数
  HXC_CAN(const HXC_CAN&)=delete;
  HXC_CAN& operator=(const HXC_CAN&)=delete;
  //删除移动构造函数
  HXC_CAN(HXC_CAN&&)=delete;
  HXC_CAN& operator=(HXC_CAN&&)=delete;
  
  HXC_CAN(){}
  virtual ~HXC_CAN(){}

  /**
   * @description: 初始化
   * @return {esp_err_t} 成功返回ESP_OK
   * @Author: qingmeijiupiao
   * @param {CAN_RATE} can_rate
   */
  virtual esp_err_t setup(CAN_RATE can_rate){
    return ESP_FAIL;
  };

  /**
   * @description: 发送CAN消息
   * @return {esp_err_t} 成功返回ESP_OK
   * @Author: qingmeijiupiao
   * @param {HXC_CAN_message_t*} message
   */
  virtual esp_err_t can_send(HXC_CAN_message_t* message){
    return ESP_FAIL;
  };

/**
 * @description: 发送CAN消息
 * @return {esp_err_t} 成功返回ESP_OK
 * @Author: qingmeijiupiao
 * @param {HXC_CAN_message_t} message
 */
  virtual esp_err_t can_send(HXC_CAN_message_t message){
    return ESP_FAIL;
  };


  /**
   * @description: 添加CAN消息接收回调,收到对应地址的消息时运行回调函数
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {int} addr //CAN消息地址
   * @param {HXC_can_feedback_func} func 回调函数
   */
  void add_can_receive_callback_func(int addr,HXC_can_feedback_func func){
    func_map[addr]=func;
  };


  /**
   * @description: 移除CAN消息接收回调函数
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {int} addr
   */
  void remove_can_receive_callback_func(int addr){
    if(!(exist_can_receive_callback_func(addr))){return;}
    func_map.erase(addr);
  };


  /**
   * @description: 判断CAN消息接收回调函数是否存在
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {int} addr CAN消息地址
   */
  bool exist_can_receive_callback_func(int addr){
    return func_map.find(addr)!=func_map.end();
  };
  protected:
  std::map<int,HXC_can_feedback_func> func_map;//can回调map
  bool is_setup=false;
  CAN_RATE can_rate;
};

class HXC_TWAI:public HXC_CAN{
  public:
  /**
   * @description: TWAI构造函数,默认为HXC开发板A的引脚
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {uint8_t} tx 连接can收发芯片TX引脚的IO号
   * @param {uint8_t} rx 连接can收发芯片RX引脚的IO号
   * @param {CAN_RATE} rate CAN速率
   */
  HXC_TWAI(uint8_t tx=8, uint8_t rx=18,CAN_RATE rate=CAN_RATE_1MBIT){
    TX_PIN=tx;
    RX_PIN=rx;
    can_rate=rate;
  };

  /**
   * @description: TWAI初始化
   * @return {esp_err_t}  成功返回ESP_OK
   * @Author: qingmeijiupiao
   */
  esp_err_t setup(){
    if(is_setup==true){return ESP_OK;};
    //总线速率,1Mbps
    static twai_timing_config_t t_config;
    //设置总线速率
    switch (this->can_rate){
      case CAN_RATE_1MBIT:
        t_config= TWAI_TIMING_CONFIG_1MBITS();
        break;
      case CAN_RATE_800KBIT:
        t_config= TWAI_TIMING_CONFIG_800KBITS();
        break;
      case CAN_RATE_500KBIT:
        t_config= TWAI_TIMING_CONFIG_500KBITS();
        break;
      case CAN_RATE_250KBIT:
        t_config= TWAI_TIMING_CONFIG_250KBITS();
        break;
      case CAN_RATE_125KBIT:
        t_config= TWAI_TIMING_CONFIG_125KBITS();
        break;
      case CAN_RATE_100KBIT:
        t_config= TWAI_TIMING_CONFIG_100KBITS();
        break;
      default:
        t_config= TWAI_TIMING_CONFIG_1MBITS();
        break;
    };
    //滤波器设置,接受所有地址的数据
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //总线配置
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(TX_PIN), gpio_num_t(RX_PIN), TWAI_MODE_NO_ACK);

    //传入驱动配置信息
    twai_driver_install(&g_config, &t_config, &f_config);
    //CAN驱动启动
    auto status = twai_start();
    if(status!=ESP_OK){return status;};
    //创建任务
    xTaskCreate(twai_feedback_update_task,"twai_fb",4096,this,5,&twai_fb_handle);//can反馈任务
    is_setup=true;
    return ESP_OK;
  };

  /**
   * @description: 发送CAN消息
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {HXC_CAN_message_t*} message CAN消息
   */
  esp_err_t can_send(HXC_CAN_message_t* message) override {
    twai_message_t twai_message;
    twai_message.extd=message->extd;
    twai_message.rtr=message->rtr;
    twai_message.self=message->self;
    twai_message.identifier=message->identifier;
    twai_message.data_length_code=message->data_length_code;
    for(int i=0;i<message->data_length_code;i++){
        twai_message.data[i]=message->data[i];
    }
    return twai_transmit(&twai_message,portMAX_DELAY);
  };


  /**
   * @description: 发送CAN消息
   * @return {*}
   * @Author: qingmeijiupiao
   * @param {HXC_CAN_message_t} message CAN消息
   */
  esp_err_t can_send(HXC_CAN_message_t message) override {
    twai_message_t twai_message;
    twai_message.extd=message.extd;
    twai_message.rtr=message.rtr;
    twai_message.self=message.self;
    twai_message.identifier=message.identifier;
    twai_message.data_length_code=message.data_length_code;
    for(int i=0;i<message.data_length_code;i++){
        twai_message.data[i]=message.data[i];
    }
    return twai_transmit(&twai_message,portMAX_DELAY);
  };

  
  /**
   * @description: 停止接收CAN消息
   * @return {*}
   * @Author: qingmeijiupiao
   */
  void stop_receive(){
    if(twai_fb_handle==nullptr){return;};
    vTaskDelete(twai_fb_handle);
    twai_fb_handle=nullptr;
  }

  /**
   * @description: 停止之后使用该函数恢复接收CAN消息
   * @return {*}
   * @Author: qingmeijiupiao
   */
  void resume_receive(){
    if(twai_fb_handle!=nullptr){return;};
    xTaskCreate(twai_feedback_update_task,"twai_fb",4096,this,5,&twai_fb_handle);//can反馈任务
  }

  /**
   * @description:  判断此时是否在持续接收CAN消息
   * @return {bool} true:持续接收CAN消息  false:停止接收CAN消息
   * @Author: qingmeijiupiao
   */
  bool get_receive_status(){
    return twai_fb_handle!=nullptr;
  };
  protected:

  /**
   * @description: TWAI接受数据的线程，接收到数据后转换成HXC_CAN_message，然后调用回调函数
   */
  static void twai_feedback_update_task(void* n){
    HXC_TWAI* twai=(HXC_TWAI*)n;
    twai_message_t Twai_message;
    
    auto To_HXC_CAN_message_t=[&](twai_message_t* twai_message){
        twai->RX_message_buf.extd=twai_message->extd;
        twai->RX_message_buf.rtr=twai_message->rtr;
        twai->RX_message_buf.self=twai_message->self;
        twai->RX_message_buf.identifier=twai_message->identifier;
        twai->RX_message_buf.data_length_code=twai_message->data_length_code;
        for(int i=0;i<twai_message->data_length_code;i++){
            twai->RX_message_buf.data[i]=twai_message->data[i];
        }
    };
    
    while (1){
        //接收CAN上数据
        ESP_ERROR_CHECK(twai_receive(&Twai_message, portMAX_DELAY));
        //查看是否为需要的can消息地址，如果是就调用函数
        if(twai->exist_can_receive_callback_func(Twai_message.identifier)){
            //将twai_message转换成HXC_CAN_message
            To_HXC_CAN_message_t(&Twai_message);
            //调用回调函数
            twai->func_map[twai->RX_message_buf.identifier](&twai->RX_message_buf);
        }
    }
  };
  
  TaskHandle_t twai_fb_handle;//twai反馈任务句柄
  uint8_t TX_PIN,RX_PIN;
  HXC_CAN_message_t RX_message_buf;//接收CAN消息缓存
};

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