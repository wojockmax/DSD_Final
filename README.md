# DSD_Final
如檔案夾名，此code已通過助教給的baseline(no/has hazard)，但是其他的extension code我就不保證了(累)。本資料夾除了程式碼之外，
在Multicycle_MIPS的資料夾內含一word檔說明對control的設定，如果control的部分有疑問的話可以去翻閱。以及在src內有架構圖，不過大家也都看過了(也沒完全按那個接)。

由於我將程式碼分開寫(想說這樣會比較好debug)，因此在compile的時候請要在後面打cache.v跟MIPS_Pipeline.v(因為助教目前不給改CHIP.v QAQ)，如下：

no hazard : ncverilog Final_tb.v CHIP.v slow_memory.v MIPS_Pipeline.v cache.v +define+noHazard +access+r

has hazard : ncverilog Final_tb.v CHIP.v slow_memory.v MIPS_Pipeline.v cache.v +define+hasHazard +access+r

本次的設計與架構圖不同之處是把Forwarding unit 提前至ID區處理，使得Branch, Jump, JumpReg只需要有一個Flush而已(減少cycle數)，但就要在critical path上要付出代價，不過結果如何，我就不能保證了。此外，我已經用dc確認過沒有latch了，應該是可以合成的(但我沒實際合)。
cache的部分是使用two-way跟write back模式，然後有稍微優化一下。

在本資料夾除了Pipeline的code之外，還有Single_MIPS的code(DSDHW3.v)，不過功能有閹割過(因為比較方便)，但足夠可以執行hasHazard，可以做為對照用，指令如下：

no hazard : ncverilog Final_tb.v CHIP.v slow_memory.v DSDHW3.v cache.v +define+noHazard +access+r

has hazard : ncverilog Final_tb.v CHIP.v slow_memory.v DSDHW3.v cache.v +define+hasHazard +access+r

我想說的大概就是這樣，還有debug真的有夠累的....

http://www.1111boss.com.tw/new/franchisee/?sTradeCht=&sTrade=&sMoney=&sKeys=%E9%9B%9E%E6%8E%92&btn_Search=1
你們覺得我要去加盟哪一間雞排店啊？麻煩給個建議，感謝。
