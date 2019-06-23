 #include <SoftwareSerial.h> // 引用程式庫
// 定義連接藍牙模組的序列埠
SoftwareSerial BT2(12,13); // 接收腳, 傳送腳
String val = ""; // 儲存接收資料的變數
char data;
bool receive = false;
void setup() {
Serial.begin(57600); // 與電腦序列埠連線
Serial.println("BT is ready!");
// 設定藍牙模組的連線速率
// 如果是HC05，請改成38400
// 如果是HC06，請改成9600
BT2.begin(57600);
}
void loop() {
// 若收到「序列埠監控視窗」的資料，則送到藍牙模組
if (Serial.available()) {
val = Serial.readString();
BT2.print(val+"\n");
}
else
  BT2.print("\n");
delay(50);
// 若收到藍牙模組的資料，則送到「序列埠監控視窗」
while (BT2.available()) {
  receive = true;
  data = BT2.read();
  val += data;
  }
  if(receive)
  {
    receive = false;
    Serial.println(val);
    val = "";
  }
}
