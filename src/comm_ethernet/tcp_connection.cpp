#include <iostream>
#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace tofcore {

typedef std::vector<uint8_t> Packet;

TcpConnection::TcpConnection(boost::asio::io_service& ioService)
  : state(STATE_DISCONNECTED), socket(ioService), resolver(ioService) {
  connect();
}

TcpConnection::~TcpConnection() {
  try {
    disconnect();
  } catch (boost::system::system_error& e) {
    std::cerr << e.what() << std::endl;
  }
}

void TcpConnection::sendCommand(const std::vector<uint8_t>& data) {

  std::vector<uint8_t> answer;
  this->sendCommand(data, answer);
}


void TcpConnection::sendCommand(const std::vector<uint8_t> &data, std::vector<uint8_t>& payload)
{
  if (!isConnected()) return;

  uint32_t data_len = data.size();

  std::ostringstream os;
  os << START_MARKER;
  os << static_cast<uint8_t>((data_len >> 24) & 0xff);
  os << static_cast<uint8_t>((data_len >> 16) & 0xff);
  os << static_cast<uint8_t>((data_len >>  8) & 0xff);
  os << static_cast<uint8_t>((data_len >>  0) & 0xff);
  for (uint32_t i = 0; i < data_len; ++i) {
    os << static_cast<uint8_t>(data[i]);
  }
  os << END_MARKER;

  boost::system::error_code error;
  socket.write_some(boost::asio::buffer(os.str(), os.tellp()), error);
  if (error) {
    throw boost::system::system_error(error);
  }

  //Read the start mark, the answer type and size info (6 bytes), that will give us the information needed to read the complete payload. 
  Packet buf(8);

  auto len = boost::asio::read(socket, boost::asio::buffer(buf));
  assert(len == buf.size());
  uint32_t start_marker = ::ntohl(*reinterpret_cast<uint32_t*>(buf.data()+0));
  assert(start_marker == 0xFFFFAA55);
  // assert(buf[0] == 0xFF);
  // assert(buf[1] == 0xFF);
  // assert(buf[2] == 0xAA);
  // assert(buf[3] == 0x55);
  uint32_t payload_size = ::ntohl(*reinterpret_cast<uint32_t*>(buf.data()+4));

  //Now read the payload
  payload.resize(payload_size);
  len = boost::asio::read(socket, boost::asio::buffer(payload));
  assert(len == payload_size);

  //Read the end mark
  buf.resize(4);
  len = boost::asio::read(socket, boost::asio::buffer(buf));
  assert(len == buf.size());
  assert(::ntohl(*reinterpret_cast<uint32_t*>(buf.data())) == 0xFFFF55AA);
}


void TcpConnection::connect() {
  if (isConnected()) return;

  updateState(STATE_CONNECTING);
  tcp::resolver::query query(HOST, PORT);
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iterator != end) {
    socket.close();
    socket.connect(*endpoint_iterator++, error);
  }
  if (error) {
    throw::boost::system::system_error(error);
  }
  updateState(STATE_CONNECTED);
}

void TcpConnection::disconnect() {
  if (isDisconnected()) return;

  updateState(STATE_CLOSING);

  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
  if (error) {
    revertState();
    throw boost::system::system_error(error);
  }
  updateState(STATE_DISCONNECTED);
}

void TcpConnection::waitAck() {
  Packet buf(ACK_BUF_SIZE);
  boost::system::error_code error;

  this->updateState(STATE_WAIT_ACK);
  size_t len = socket.read_some(boost::asio::buffer(buf), error);
  (void)len;
  if (error) {
    throw boost::system::system_error(error);
  }
  this->revertState();
}

void TcpConnection::updateState(State state_) const {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() const {
  state = previousState;
}

bool TcpConnection::isConnected() const {
  return state == STATE_CONNECTED;
}

bool TcpConnection::isDisconnected() const {
  return state == STATE_DISCONNECTED;
}

} // end namespace tofcore
