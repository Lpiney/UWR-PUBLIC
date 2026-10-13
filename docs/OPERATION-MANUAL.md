# UWR 比賽操作手冊

按**比賽手冊的任務編號**組織。每個任務一節，寫清楚四件事：

1. **比賽手冊要求什麼**——引用原文，不改寫
2. **我們用什麼滿足它**——對應到程式碼的哪裡
3. **現場怎麼操作**——照著做
4. **怎麼確認成功**——裁判看到什麼、你看什麼

跟[操作指南](GUIDE.md)的分工：指南是「第一次怎麼把東西裝起來、燒上去、讓馬達轉」，
這份是「比賽當天每個任務怎麼打」。程式細節在程式碼註解裡，這裡不重複。

## 任務總表

| 任務 | 分數 | 對照狀態 |
|---|---|---|
| [Mission 1: Communication Pairing](#mission-1-communication-pairing20-分) | 20 | ✅ 已對照 |
| Mission 2.1: AprilTag Scanning | 40 | 待檢查 |
| Mission 2.2: Chamber Docking | 20 | 待檢查 |
| Mission 3: Line Following | 30 | 待檢查 |
| Mission 4: Object Retrieval | 40 | 待檢查 |
| Mission 5: Pole Color Recognition and Hitting | 50 | 待檢查 |

> 分數合計 200 分，與手冊的 Product Demonstration 總分一致。

---

# Mission 1: Communication Pairing（20 分）

## 一、比賽手冊要求什麼

> both the connection to the Wi-Fi network and the retrieval of data from the
> HTTP server are operations that must be carried out **by the ROV itself**.

> An award of 20 points will be given to each team whose successful retrieval
> of the data is followed by **the presentation of that same data to the
> referee**.

> It must be clearly understood that the retrieval of the data is to be
> performed by the **microcontroller unit, or MCU, that is located on the ROV**.

> Any retrieval of the data that is carried out by means of other devices or
> other methods — whether a **laptop, a phone, or a browser** is used for that
> purpose — will not be rewarded with points, and this remains the case **even
> when the data is shown or displayed afterward**.

> The WiFi SSID is **UR_Field**, and its pairing Passcode is **12345678**.

> After successfully connecting the tower, a message
> `[Congratulations!!! This is the WiFi API task message for the fall training.]`
> will be sent by the tower periodically.

> **The IP address of the tower** will be released during the competition.

**一句話總結**：ROV 自己連 Wi-Fi、自己抓資料，然後把那串字擺在裁判面前。
筆電只能當顯示器，不能碰網路。

## 二、我們用什麼滿足它

| 手冊要求 | 程式 | 位置 |
|---|---|---|
| Wi-Fi 連線由 ROV 自己做 | `WiFi.begin(ssid, pass)` 跑在 ESP32 上 | [mission1.h:142](../firmware/ESP32/src/mission1.h) |
| HTTP 取資料由 ROV 自己做 | `HTTPClient::GET()` 跑在 ESP32 上 | [mission1.h:109](../firmware/ESP32/src/mission1.h) |
| 筆電不得參與取資料 | 筆電端**完全沒有 HTTP 能力**（沒有 `requests`／`urllib`／`socket`），`requirements.txt` 也沒有網路庫 | [requirements.txt](../host/requirements.txt) |
| 取到後展示給裁判 | 按**手柄上的 X** → 左下角 `DATA` 顯示原文 | [console.py:761](../host/console.py) |
| SSID `UR_Field` / `12345678` | 編譯進去的預設值，與手冊一致 | [params.h:23](../firmware/ESP32/src/params.h) |
| 塔的位址當天公布 | `P url <位址>` 執行期設定，存 NVS | [params.h:128](../firmware/ESP32/src/params.h) |
| 塔週期性送訊息 | 每 2 秒輪詢一次（`pollMs`） | [params.h:32](../firmware/ESP32/src/params.h) |

**「筆電不能碰網路」這條是架構保證的，不是靠自律。** 筆電端沒有任何發出 HTTP 請求的能力，
連裝都沒裝。所以不可能誤犯。

### 為什麼取資料一定要在板子上

不只是規則問題。[mission1.h:10](../firmware/ESP32/src/mission1.h) 說明了技術理由：
HTTP 請求最長會阻塞 2.5 秒。這條路徑跑在 **core 0 的獨立 task**，控制迴圈獨佔 core 1。
如果放在主迴圈，這 2.5 秒送不出驅動指令，**斷流保護會把馬達切回中立**——
每次網路打嗝螺旋槳就停一下。

## 三、現場怎麼操作

### 步驟 1：賽前設定塔的位址（調整期間做，不要等到進站）

**這一步一定要在啟動 `console.py` 之前做。** `console.py` 會佔住序列埠，
跑起來之後**沒有任何介面可以送 `P` 指令**。

```
① 先不要開 console.py
② 用序列埠監視器連上板子（115200）
③ 打：  P url 192.168.4.1
④ 看到：OK P url = http://192.168.4.1        ← 看這行，確認補上的協定
⑤ 關掉監視器（一定要關，否則 console.py 開不了埠）
⑥ 現在才啟動 console.py
```

> **少了 `http://` 也沒關係。** 手冊承諾給你的是 IP 不是 URL，
> 所以直接打 `192.168.4.1` 是最自然的輸入。程式會自動補上 `http://`，
> 並在 `OK` 那行把補完的結果印給你確認。
>
> 但**如果位址本來就有 `://`，程式一個字都不會改**。

> **位址存在 NVS，斷電和重新燒錄都不會掉。** 一整天只要設一次。
> 除非裁判說塔換了位址，否則不用再碰。

進站前檢查一次就好：

```
G url        →   OK url = http://192.168.4.1
```

### 步驟 2：進站佈置（5 分鐘）

```
① USB 線接上筆電（板子同時從 USB 取電）
② 啟動操作台：
      python3 host/console.py
③ 確認左上角出現綠點 ● LINK
```

> **開序列埠時板子可能會重啟一次。** 程式等不到遙測時會提示這件事。
> 重啟不是故障，但重啟後有 **3 秒開機預熱期**，這段時間板子不接受指令。
> 所以把 `P url` 留在**進站前**送完，進站之後就不要再插拔序列埠。
>
> ⚠️ 這一點**沒有在真板上驗證過**。第一次接上時留意一下：如果板子重啟了，
> 之後就照「進站前設定」的順序做；如果沒重啟，這條可以忽略。

### 步驟 3：連線（板子自己做，你不用管）

板子**一上電就自動連** UR_Field，不需要按任何鍵、送任何指令。

看左下角面板：

```
MODE  Mission 1
wifi CONNECTED   rssi -58
ip   192.168.4.7
http 200
DATA [Congratulations!!! This is the WiFi API task message for the fall training.]
```

| 欄位 | 意思 |
|---|---|
| `wifi CONNECTED` | 已連上塔。其他值：`IDLE`／`CONNECTING`／`LOST` |
| `rssi` | 訊號強度。−50 好，−80 快斷了 |
| `ip` | 板子自己拿到的 IP。顯示 `-` 表示還沒連上 |
| `http` | 最後一次請求的結果。`200` 成功；負數是錯誤碼 |

### 步驟 4：取資料並給裁判看

```
按手柄上的 X  →  程式自動送一條 F，板子立刻抓一次
              →  左下角 DATA 那行就是塔送來的原文
```

**按手柄上的 X 會強制立刻抓一次**，不等 2 秒的輪詢週期。
這樣裁判看到的是**當下剛抓的**，不是上一次輪詢留下的。

## 四、怎麼確認成功

裁判要看的是這一行，而且要是**塔送來的原文**：

```
[Congratulations!!! This is the WiFi API task message for the fall training.]
```

畫面上長這樣（自動換行，不會被切掉）：

```
DATA [Congratulations!!! This is the WiFi API task
     message for the fall training.]
```

**成功的三個條件，缺一不可**：

1. 左上角綠點 ● LINK
2. `wifi CONNECTED` 且 `http 200`
3. `DATA` 顯示的是上面那串原文

## 五、出錯時

| 現象 | 檢查 |
|---|---|
| `wifi CONNECTING` 一直不動 | 板子離塔太遠，或密碼不對。先 `G ssid` / `G pass` 確認是 `UR_Field` / `12345678` |
| `wifi LOST` 反覆跳 | **不要反覆重啟板子。** 家用路由器對短時間內反覆連線有防抖，越急越連不上。把間隔調大：`P wifiRetryMs 30000` |
| `http 200` 但 `DATA` 空的 | 塔回了 200 但內容是空的。按手柄上的 X 再抓一次 |
| `http -1` 之類的負數 | 連不上伺服器。確認 `G url` 是對的位址 |
| `EV url not set` | 沒設位址。回到步驟 1 |
| `EV http: bad URL` | 位址格式還是錯。`G url` 看實際存了什麼 |
| 連不上塔，但上次實驗室明明可以 | **實驗室留下的 Wi-Fi 設定還在 NVS 裡。** 送一條 `R` 清掉，回到手冊的預設值 |

> **`R` 會清掉 `url`。** `R` 是「恢復全部預設值」，包括網路設定。
> 清完要重新 `P url`。比賽當天開機先發 `R` 是為了清掉實驗室的殘留，
> 但如果你已經設好位址了，就不要再送 `R`。

## 六、比賽前該複查的

- [ ] `G url` 印出來的是當天裁判給的位址
- [ ] 這串位址有沒有可能是錯的——裁判給的是 IP，補上 `http://` 是對的
- [ ] 在**實驗室**先跑一次完整流程，確認畫面顯示正常
- [ ] 確認筆電上沒有其他程式佔著序列埠（序列埠監視器、另一個 `console.py`）
- [ ] 接上板子時留意**開序列埠會不會讓板子重啟**，決定進站後的插拔順序
- [ ] Mission 1 的完整流程**還沒在真板 + 真塔上跑過**。實驗室那次要當成第一次

---

# Mission 2.1: AprilTag Scanning（40 分）

待檢查。

---

# Mission 2.2: Chamber Docking（20 分）

待檢查。

---

# Mission 3: Line Following（30 分）

待檢查。

---

# Mission 4: Object Retrieval（40 分）

待檢查。

---

# Mission 5: Pole Color Recognition and Hitting（50 分）

待檢查。
