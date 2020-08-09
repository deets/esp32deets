// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once
#include <atomic>
#include <memory>

struct NonCopyable {
    NonCopyable & operator=(const NonCopyable&) = delete;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable() = default;
};

template<typename T, int N>
class RingBuffer
{
public:
  class Reader : public NonCopyable
  {
  public:
    Reader(Reader&& other)
      : _buffer(other._buffer)
      , _read(other._read.load())
      , _total_reads(other._total_reads.load())
    {
      _buffer = nullptr;
    }

    bool empty() const
    {
      return count() == 0;
    }

    uint32_t count() const
    {
      return (_buffer->_write + N - _read) % N;
    }

    const T& read()
    {
      const auto& v = _buffer->_items[_read];
      _read = (_read + 1) % N;
      _total_reads++;
      return v;
    }

    template<typename F>
    void consume(F func)
    {
      while(!empty())
      {
        func(read());
      }
    }

    bool overrun() const
    {
      return overrun_count() > N;
    }

    int overrun_count() const
    {
      return (_buffer->_total_writes - _total_reads) / N;
    }

  private:
    Reader(RingBuffer* buffer)
      : _buffer(buffer)
      , _read(buffer->_write.load())
      , _total_reads(0)
    {}
    RingBuffer* _buffer;
    std::atomic<uint32_t> _read, _total_reads;
    friend RingBuffer;
  };

  RingBuffer() : _write(0), _total_writes(0)
  {
  }

  Reader reader()
  {
    return Reader(this);
  }

  void append(const T& value)
  {
    _items[_write] = value;
    _write = (_write + 1) % N;
    _total_writes++;
  }

private:
  std::atomic<uint32_t> _write, _total_writes;
  T _items[N];
};
