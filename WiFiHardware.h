#include <WiFi.h>
IPAddress server(192,168,8,2);
WiFiClient client;

class WiFiHardware {
  public:
  
  WiFiHardware() {};
  
  void init(){
    client.connect(server, 11411);
  }

  int read() {
    return client.read();
  }

  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
  
  unsigned long time() {
     return millis();
  }
};
