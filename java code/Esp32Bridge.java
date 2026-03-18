package frc.robot; // gerektiği gibi değiştirin

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Basit ESP32 SPI bridge istemcisi (RoboRIO tarafı).
 *
 * Protokol:
 *  - Transaction A (master -> slave): gönderilen buffer[0] = 0x01 (CMD_REQUEST_EULER), uzunluk TRANS_SZ
 *  - Kısa bekleme (ESP32 cevabı hazırlar)
 *  - Transaction B (master -> slave): gönderilen TRANS_SZ dummy bayt; gelen yanıtın ilk 12 baytı
 *    3 adet float32 (little-endian) olarak yorumlanır: x,y,z
 */
public class Esp32Bridge {
  private final SPI spi;
  private final DigitalOutput csManual; // optional, null ise SPI kütüphanesi CS'yi otomatik yönetir
  private static final int TRANS_SZ = 16; // ESP32 ile eşleşmeli
  private static final int CMD_REQUEST_EULER = 0x01;
  private static final double PREP_DELAY_SECONDS = 0.001; // 1 ms

  public Esp32Bridge() {
    this(null);
  }

  /**
   * @param csDioChannel null ise otomatik CS kullan. Integer DIO kanalı verilirse, manuel CS kontrolü yapılır.
   */
  public Esp32Bridge(Integer csDioChannel) {
    spi = new SPI(SPI.Port.kMXP);
    spi.setClockRate(1_000_000); // 1 MHz öneri (ESP tarafı max 3MHz)
    // WPILib SPI sınıfı doğrudan bit-order ayarı sağlamaz; default MSB-first kullanılır.
    // Bu yüzden setMSBFirst() çağrısı gereksiz/yanlıştır ve kaldırıldı.
    spi.setMode(SPI.Mode.kMode0);

    if (csDioChannel != null) {
      csManual = new DigitalOutput(csDioChannel);
      // Varsayılan olarak deassert edilmiş (aktif düşük CS varsayımı). Bağlantınıza göre invert gerekebilir.
      csManual.set(true);
    } else {
      csManual = null;
    }
  }

  /**
   * ESP32'den Euler açılarını okur.
   *
   * @return float[3] = {roll(x), pitch(y), yaw(z)} veya hata durumunda null
   */
  public float[] readEuler() {
    byte[] tx1 = new byte[TRANS_SZ];
    byte[] rx1 = new byte[TRANS_SZ];
    tx1[0] = (byte) CMD_REQUEST_EULER;

    // Transaction 1: gönder komut (ESP'ye yanıt hazırlaması için)
    if (csManual != null) csManual.set(false); // CS aktif (aktif düşük varsayımı)
    int rc1 = spi.transaction(tx1, rx1, TRANS_SZ);
    if (csManual != null) csManual.set(true); // CS deassert
    if (rc1 != TRANS_SZ) {
      // Başarısız veya eksik transfer
      return null;
    }

    // Kısa bekleme; ESP32 bu süre içinde cevabı hazırlar
    Timer.delay(PREP_DELAY_SECONDS);

    // Transaction 2: karşı taraftan hazır veriyi oku (MASTER clocks out slave TX)
    byte[] tx2 = new byte[TRANS_SZ]; // dummy (sıfırlar)
    byte[] rx2 = new byte[TRANS_SZ];
    if (csManual != null) csManual.set(false);
    int rc2 = spi.transaction(tx2, rx2, TRANS_SZ);
    if (csManual != null) csManual.set(true);
    if (rc2 != TRANS_SZ) {
      return null;
    }

    // rx2[0..11] => 3 float32 little-endian
    ByteBuffer bb = ByteBuffer.wrap(rx2).order(ByteOrder.LITTLE_ENDIAN);
    try {
      float x = bb.getFloat(0);
      float y = bb.getFloat(4);
      float z = bb.getFloat(8);
      return new float[] {x, y, z};
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void close() {
    spi.close();
    if (csManual != null) csManual.close();
  }
}