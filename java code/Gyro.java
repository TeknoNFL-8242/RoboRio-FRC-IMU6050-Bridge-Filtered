package frc.robot;

/**
 * Basit Gyro wrapper.
 *
 * Inputs:
 *  - Hardware gyro sensor (IMU)
 * Outputs:
 *  - getYaw() -> Robot dönüş kontrolünde veya odometriye beslenir
 *
 * Kullanım örn:
 *   Gyro gyro = new Gyro();
 *   double heading = gyro.getYaw();
 *   gyro.reset();
 */
public class Gyro {

	/** Mevcut açı (derece) döndürür. Stub: gerçek sensörle bağlanmalı. */
	public double getYaw() {
		// TODO: gerçek IMU'dan oku
		return 0.0;
	}

	/** Gyro'yu sıfırlar. */
	public void reset() {
		// TODO: gerçek IMU reset çağrısı
	}
}
