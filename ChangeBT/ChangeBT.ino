#include <SoftwareSerial.h> // 引用程式庫
// 定義連接藍牙模組的序列埠
SoftwareSerial BTM(8, 9); // 接收腳, 傳送腳
//SoftwareSerial BTS(10, 11);
char val; // 儲存接收資料的變數
void setup() {
  Serial.begin(57600); // 與電腦序列埠連線
  Serial.println("BT is ready!");
  // 設定藍牙模組的連線速率
  // 如果是HC05，請改成38400
  // 如果是HC06，請改成9600
  BTM.begin(57600);
}
void loop() {
  if (Serial.available()) {
  val = Serial.read();
  BTM.print(val);
  }
  if (BTM.available()) {
  val = BTM.read();
  Serial.print(val);
  }
}
