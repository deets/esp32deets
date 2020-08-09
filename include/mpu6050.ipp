// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#include <esp_log.h>


template<typename T>
void MPU6050::consume_fifo(T callback)
{
  std::array<uint8_t, 1024> buffer;
  const auto current_fifo_count = fifo_count();
  const auto datagram_count = (current_fifo_count - (current_fifo_count % _fifo_datagram_size)) / _fifo_datagram_size;

  //ESP_LOGI("mpu", "current_fifo_count: %i, datagram_count: %i", current_fifo_count, datagram_count);
  if(datagram_count == 0)
  {
    return;
  }


  _i2c.read_from_device_register_into_buffer(
    _address, MPU6050_FIFO_RW,
    buffer.data(),
    _fifo_datagram_size * datagram_count
    );
  auto p = buffer.data();

  for(size_t i=0; i < datagram_count; ++i)
  {
    gyro_data_t entry;
    std::memset(&entry, 0, sizeof(gyro_data_t));
    p = populate_entry(p, entry);
    callback(entry);
  }
}
