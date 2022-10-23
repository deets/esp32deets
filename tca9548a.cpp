#include "deets/i2c/tca9548a.hpp"
#include <mutex>

namespace deets::i2c {

TCA9548A::WrappedBus::WrappedBus(uint8_t busno, TCA9548A &mux, I2C &bus)
  : _busno(busno)
  , _mux(mux)
  , _bus(bus)
{
}


esp_err_t TCA9548A::WrappedBus::write_byte(uint8_t address, uint8_t value) const
{
  _mux.select(_busno);
  return _bus.write_byte(address, value);
}

esp_err_t TCA9548A::WrappedBus::read_byte_from_register(uint8_t address,
                                                        uint8_t register_,
  uint8_t& res) const
{
  _mux.select(_busno);
  return _bus.read_byte_from_register(address, register_, res);
}

esp_err_t TCA9548A::WrappedBus::write_byte_to_register(uint8_t address,
                                                  uint8_t register_,
                                                  uint8_t value) const
{
  _mux.select(_busno);
  return _bus.write_byte_to_register(address, register_, value);
}

esp_err_t TCA9548A::WrappedBus::write_buffer_to_address(uint8_t address,
                                                   const uint8_t *buffer,
                                                   size_t len) const {
  _mux.select(_busno);
  return _bus.write_buffer_to_address(address, buffer, len);
}

esp_err_t TCA9548A::WrappedBus::read_from_device_register_into_buffer(uint8_t address, uint8_t register_, uint8_t* buffer, size_t length) const
{
  _mux.select(_busno);
  return _bus.read_from_device_register_into_buffer(address, register_, buffer, length);
}

esp_err_t TCA9548A::WrappedBus::read_from_address_into_buffer(uint8_t address, uint8_t *buffer,
                                   size_t length) const
{
  _mux.select(_busno);
  return _bus.read_from_address_into_buffer(address, buffer, length);
}

std::lock_guard<TCA9548A::WrappedBus::mutex_type> TCA9548A::WrappedBus::lock()
{
  return _bus.lock();
}

std::vector<uint8_t> TCA9548A::WrappedBus::scan() const
{
  _mux.select(_busno);
  return _bus.scan();
}


TCA9548A::TCA9548A(I2C &bus, uint8_t address)
    : _busses{WrappedBus{0, *this, bus}, WrappedBus{1, *this, bus},
              WrappedBus{2, *this, bus}, WrappedBus{3, *this, bus},
              WrappedBus{4, *this, bus}, WrappedBus{5, *this, bus},
              WrappedBus{6, *this, bus}, WrappedBus{7, *this, bus}},
      _address(address), _selected_bus(255), _bus(bus) {}

I2C &TCA9548A::bus(uint8_t busno) { return _busses[busno]; }

void TCA9548A::select(uint8_t busno)
{
  if(_selected_bus != busno)
  {
    auto lock = _bus.lock();
    _bus.write_byte(_address, 1 << busno);
    _selected_bus = busno;
  }
}

} // namespace deets::i2c
