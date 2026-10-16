# Windows：開序列埠監視器送指令

比賽當天要改板子的設定（塔的位址、Wi-Fi）時用的。**Windows 專用**，
其他步驟見[操作指南](../GUIDE.md)和[比賽操作手冊](../OPERATION-MANUAL.md)。

---

## 先搞清楚一件事

`P url 192.168.4.1:8080` **不是**終端機指令。

| | 在哪裡打 | 例子 |
|---|---|---|
| **Shell 指令** | 終端機 | `pio run -t upload`、`git push` |
| **板子指令** | **序列埠監視器** | `P url …`、`G ssid`、`S` |

終端機只是視窗。序列埠監視器是**跑在終端機裡面的一個程式**，負責跟板子對話。
所以你要先用終端機把監視器**開起來**，然後才在裡面打板子指令。

---

## 步驟 1：開一個終端機

VS Code 裡按 **`` Ctrl+` ``**（Ctrl 加鍵盤左上角 `` ` ``），
或用選單 **Terminal → New Terminal**。

預設是 **PowerShell**，下面所有指令都能直接貼。

> 想用命令提示字元（cmd）也可以，指令一樣。

---

## 步驟 2：找出板子在哪個 COM 埠

```powershell
pio device list
```

會印出像這樣的清單（這是你機器上實際出現過的）：

```
COM4 | 透過藍牙連結的標準序列 (COM4)
COM5 | USB Serial Port (COM5)
```

**看描述，不要看編號。** Windows 的埠名只有 `COM` 加數字，看不出是什麼裝置。

| 描述 | 是不是板子 |
|---|---|
| `USB Serial Port` | ✅ FTDI 橋接晶片 |
| `USB-SERIAL CH340` | ✅ CH340 橋接晶片 |
| `Silicon Labs CP210x` | ✅ CP210x 橋接晶片 |
| `USB Serial Device` | ✅ ESP32 自己的 USB |
| **`透過藍牙連結的標準序列`／`Bluetooth`** | ❌ **不是**，是電腦的藍牙 |
| `Communications Port` | ❌ 主機板內建 |

> ⚠️ **整排都是藍牙的情況很常見。** 那是電腦自己的藍牙序列埠，跟板子完全無關。
> 選錯的話打什麼都沒反應，而且不會報錯。

**記下你的埠號**（假設是 `COM5`），下一步要用。

### 如果 `pio` 被 Windows 擋掉

改用 pyserial：

```powershell
python -m serial.tools.list_ports
```

```
COM4
COM5
```

> **這個指令預設只印埠名，不印描述**，所以看不出哪個是板子。
> 加 `-v` 才會多印出來（描述和硬體 ID 各佔一行）：
>
> ```powershell
> python -m serial.tools.list_ports -v
> ```
>
> ```
> COM5
>     desc: USB Serial Port
>     hwid: USB VID:PID=0403:6001 SER=A5069RR4
> COM4
>     desc: 透過藍牙連結的標準序列 (COM4)
>     hwid: BTHENUM\{...}
> ```

---

## 步驟 3：開序列埠監視器

### 方法 A：VS Code 的 PlatformIO（推薦，最不會出問題）

1. 左側點 **PlatformIO 圖示**（螞蟻頭）
2. 展開 **`firmware`** → **`PROJECT TASKS`**
3. 點 **`Monitor`**

它會**自動找埠**，並且自動用 `platformio.ini` 裡設定的 **115200**，不用你填。

> 如果它選錯埠，讓它停掉，改用下面的方法 B 並明講埠號。

### 方法 B：命令列（可以明講埠號）

```powershell
cd firmware\ESP32
pio device monitor --port COM5 --baud 115200
```

`COM5` 換成步驟 2 查到的。

### 方法 C：`pio` 被 Windows 擋掉時用這個

如果出現 `An Application Control policy has blocked this file`，代表
Windows 的應用程式控制政策不讓 `pio.exe` 執行。改用 pyserial 的 miniterm：

```powershell
python -m pip install pyserial
```

```powershell
python -m serial.tools.miniterm COM5 115200
```

---

## 步驟 4：送指令給板子

監視器開起來之後，畫面會**自己一直跳**：

```
JOY X=+0.000 Y=+0.000 Z=+0.000 MAG=0.000 DEAD READY=1 L=1500 R=1500 V=1500 EN=0 LINK=0
ST state=READY ... wifi=CONNECTED ip=192.168.4.7 rssi=-58 gw=192.168.4.1 http=200 ...
```

**這是正常的**，板子每 50 毫秒回報一次。**在這個畫面直接打指令**，
打完整行按 **Enter**：

```
P url 192.168.4.1:8080
```

板子會回一行 `OK`：

```
OK P url = http://192.168.4.1:8080
```

**看到 `OK` 才算成功。** 沒看到就是沒送出去，或者埠選錯了。

> 打字的畫面會被遙測洗掉，看起來很亂——**沒關係，你打的字不會被吃掉**，
> 盲打完整行按 Enter 就好。

### 常用的指令

| 指令 | 作用 |
|---|---|
| `G ssid` | 看板子現在用哪個 Wi-Fi 名稱 |
| `G url` | 看板子現在用哪個位址 |
| `G` | 列出**所有**參數 |
| `S` | 看狀態（Wi-Fi、IP、HTTP 碼） |
| `P url 192.168.4.1:8080` | 設定塔的位址 |
| `P ssid 名稱` | 設定 Wi-Fi 名稱（名稱有空格也可以） |
| `P pass 密碼` | 設定 Wi-Fi 密碼 |
| `W` | 立刻重連 Wi-Fi |
| `F` | 立刻抓一次資料 |
| `B` | 重啟板子 |

> ⚠️ **不要送 `R`。** 那是「全部恢復預設值」——除了 Wi-Fi，還會把
> `neutral`、`trimL`、`trimR`、`trimV`、`fsMs` 一起清掉，
> 實驗室校準過的配平就沒了。要改哪一項就 `P` 哪一項。

> **設定會存到板子的 NVS，斷電和重新燒錄都不會掉。** 所以一天只要設一次。

---

## 步驟 5：離開監視器

按 **`Ctrl+C`**。如果沒反應，試 **`Ctrl+]`**。

**一定要離開**，不然下一個程式開不了這個埠。

---

## 步驟 6：換回操作台

```powershell
python host\console.py --port COM5
```

> ⚠️ **監視器和 `console.py` 不能同時開。** 兩個程式搶同一個埠，
> 後開的那個會失敗（`cannot open COM5`）。
>
> **所以流程永遠是：先關監視器 → 再開 console.py。**

---

## 常見問題

| 現象 | 原因 |
|---|---|
| 打指令完全沒反應 | 埠選錯了。多半是選到藍牙那個，回步驟 2 重看描述 |
| `Access is denied` / `cannot open COM5` | 埠被別的程式佔著——序列埠監視器、另一個 `console.py`，或 Arduino IDE |
| 監視器一片空白 | 板子沒通電，或選錯埠 |
| 畫面全是亂碼 | 鮑率不是 115200 |
| `pio` 一執行就被擋 | Windows 應用程式控制政策。改用方法 A 或 C |
| 打了 `P url …` 但回 `ERR unknown parameter` | 打成 `URL` 了，參數名是小寫 `url` |
| 打 `P ssid 名稱` 沒反應 | 名稱有空格是**可以**的，指令只切第一個空格。檢查是不是少了 `P` |
