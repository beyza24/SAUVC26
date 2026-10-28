# SAUVC26

ROS 2 Humble ve Gazebo üzerinde Singapore AUV Challenge (SAUVC) parkuru için su altı ROV otonomi simülasyonu: flemadan kaçma, kapıdan geçme ve dubalara temas.

Underwater ROV autonomy sim in ROS 2 Humble and Gazebo for the SAUVC course: flare dodge, gate pass, and buoy contact.

## Kapı tespiti veri seti

Bu projedeki kapıdan geçme görevi için gerçek havuz görüntülerinden oluşan bir veri seti EVREN platformunda herkese açık olarak paylaşılmıştır:

**[SAUVC Underwater Gate Detection Dataset (EVREN)](https://evren.ssyz.org.tr/datasets/u52cc5593b725/sauvc-underwater-gate-detection-dataset)**

- 120 su altı havuz görüntüsü (1920×1080), su altı videolarından kare kare çıkarıldı.
- Tek sınıf: `gate`. Kapının her turuncu direği ayrı bir sınırlayıcı kutu (BBox) ile etiketlenir.
- Topluluk katkısına açıktır; etiketleme kuralları ve örnek etiket veri setinin Benioku sayfasındadır.

Simülasyondaki kapı, renk tabanlı tespitle (`flare_detector.py`) bulunuyor. Veri seti, gerçek havuz koşullarında çalışacak öğrenme tabanlı bir kapı detektörü eğitmek için hazırlandı.

## Görev akışı

`inspect_mission` düğümü sırasıyla şunları yapar:

1. Ön kamerayla turuncu flemayı görür ve yandan kaçar.
2. Kapıdan geçer.
3. Kırmızı, sarı ve mavi dubalara temas eder.
4. Kapıdan geri geçer.
5. Başlangıç noktasına dönüp durur.

Görev parametreleri (kapı konumu, derinlik vb.) `src/abdu_control/config/mission.yaml` dosyasındadır.

## Paketler

| Paket | İçerik |
|---|---|
| `abdu_description` | ROV URDF/xacro modeli ve RViz ayarı |
| `abdu_gazebo` | Su altı dünyası, su modeli, ROV Gazebo eklentisi ve launch dosyası |
| `abdu_control` | Görev düğümü, flema/kapı/duba tespiti, 6-DOF teleop, turuncu flema renk modeli eğitimi |

## Çalıştırma

```bash
colcon build --symlink-install
./calistir.sh
```
