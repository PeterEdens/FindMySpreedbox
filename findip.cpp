//
// findip.cpp
// ~~~~~~~~
// Based on Christopher M. Kohlhoff's ping.cpp from boost, sends a ping to
// all clients on the network to find the active ip addresses and then does
// a HEAD request on a specific file we know exists on the Spreedbox to find
// all Spreedboxes on the network.
//

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <istream>
#include <iostream>
#include <ostream>

#include <Shellapi.h>
#include "icmp_header.hpp"
#include "ipv4_header.hpp"

using boost::asio::ip::icmp;
using boost::asio::deadline_timer;
namespace posix_time = boost::posix_time;

boost::asio::io_service io_service;
deadline_timer timer(io_service);

using boost::asio::ip::tcp;

extern bool DoRequest(const char* ip, const char* path);


class client
{
public:
  client(boost::asio::io_service& io_service,
      const std::string& server, const std::string& path)
    : resolver_(io_service),
      socket_(io_service),
	  server_(server)
  {
    // Form the request. We specify the "Connection: close" header so that the
    // server will close the socket after transmitting the response. This will
    // allow us to treat all data up until the EOF as the content.
    std::ostream request_stream(&request_);
    request_stream << "HEAD " << path << " HTTP/1.0\r\n";
    request_stream << "Host: " << server << "\r\n";
    request_stream << "Accept: */*\r\n";
    request_stream << "Connection: close\r\n\r\n";

    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    tcp::resolver::query query(server, "http");
    resolver_.async_resolve(query,
        boost::bind(&client::handle_resolve, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::iterator));
  }

private:
  void handle_resolve(const boost::system::error_code& err,
      tcp::resolver::iterator endpoint_iterator)
  {
    if (!err)
    {
      // Attempt a connection to each endpoint in the list until we
      // successfully establish a connection.
      boost::asio::async_connect(socket_, endpoint_iterator,
          boost::bind(&client::handle_connect, this,
            boost::asio::placeholders::error));
    }
    else
    {
      std::cout << "Error: " << err.message() << "\n";
    }
  }

  void handle_connect(const boost::system::error_code& err)
  {
    if (!err)
    {
      // The connection was successful. Send the request.
      boost::asio::async_write(socket_, request_,
          boost::bind(&client::handle_write_request, this,
            boost::asio::placeholders::error));
    }
    else
    {
      std::cout << "Error: " << err.message() << "\n";
    }
  }

  void handle_write_request(const boost::system::error_code& err)
  {
    if (!err)
    {
      // Read the response status line. The response_ streambuf will
      // automatically grow to accommodate the entire line. The growth may be
      // limited by passing a maximum size to the streambuf constructor.
      boost::asio::async_read_until(socket_, response_, "\r\n",
          boost::bind(&client::handle_read_status_line, this,
            boost::asio::placeholders::error));
    }
    else
    {
      std::cout << "Error: " << err.message() << "\n";
    }
  }

  void handle_read_status_line(const boost::system::error_code& err)
  {
    if (!err)
    {
      // Check that response is OK.
      std::istream response_stream(&response_);
      std::string http_version;
      response_stream >> http_version;
      unsigned int status_code;
      response_stream >> status_code;
      std::string status_message;
      std::getline(response_stream, status_message);
      if (!response_stream || http_version.substr(0, 5) != "HTTP/")
      {
        std::cout << "Invalid response\n";
        return;
      }
	  if (status_code == 301)
	  {
		  		  // found
		  std::cout << "found";
		  
		  std::string url = "https://";
		  url += server_;
		  ShellExecuteA(0, 0, url.c_str(), 0, 0 , SW_SHOW );
		  return;
	  }
      else if (status_code != 200)
      {
        std::cout << "Response returned with status code ";
        std::cout << status_code << "\n";
        return;
      }

      // Read the response headers, which are terminated by a blank line.
      boost::asio::async_read_until(socket_, response_, "\r\n\r\n",
          boost::bind(&client::handle_read_headers, this,
            boost::asio::placeholders::error));
    }
    else
    {
      std::cout << "Error: " << err << "\n";
    }
  }

  void handle_read_headers(const boost::system::error_code& err)
  {
    if (!err)
    {
      // Process the response headers.
      std::istream response_stream(&response_);
      std::string header;
      while (std::getline(response_stream, header) && header != "\r")
        std::cout << header << "\n";
      std::cout << "\n";

      // Write whatever content we already have to output.
      if (response_.size() > 0)
        std::cout << &response_;

      // Start reading remaining data until EOF.
      boost::asio::async_read(socket_, response_,
          boost::asio::transfer_at_least(1),
          boost::bind(&client::handle_read_content, this,
            boost::asio::placeholders::error));
    }
    else
    {
      std::cout << "Error: " << err << "\n";
    }
  }

  void handle_read_content(const boost::system::error_code& err)
  {
    if (!err)
    {
      // Write all of the data that has been read so far.
      std::cout << &response_;

      // Continue reading remaining data until EOF.
      boost::asio::async_read(socket_, response_,
          boost::asio::transfer_at_least(1),
          boost::bind(&client::handle_read_content, this,
            boost::asio::placeholders::error));
    }
    else if (err != boost::asio::error::eof)
    {
      std::cout << "Error: " << err << "\n";
    }
  }

  tcp::resolver resolver_;
  tcp::socket socket_;
  boost::asio::streambuf request_;
  boost::asio::streambuf response_;
  std::string server_;
};

class pinger
{
public:
  pinger(boost::asio::io_service& io_service, const char* destination)
    : resolver_(io_service), socket_(io_service, icmp::v4()),
      timer_(io_service), sequence_number_(0), num_replies_(0)
  {
    start_send();
    start_receive();
  }

  ~pinger()
  {
	  for (int count = 0; count < clients_.size(); count++)
	  {
		  delete clients_[count];
	  }
	  clients_.clear();
  }

private:
	std::string getip() 
	{
		using boost::asio::ip::tcp;    

		boost::asio::io_service io_service;
		tcp::resolver resolver(io_service);
		tcp::resolver::query query(boost::asio::ip::host_name(), "");
		tcp::resolver::iterator iter = resolver.resolve(query);
		tcp::resolver::iterator end; // End marker.
		while (iter != end)
		{
			tcp::endpoint ep = *iter++;
			if (ep.address().is_v4())
			{
				return ep.address().to_string();
			}
		}
	}

  void start_send()
  {
    std::string body("\"Hello!\" from Asio ping.");

    // Create an ICMP header for an echo request.
    icmp_header echo_request;
    echo_request.type(icmp_header::echo_request);
    echo_request.code(0);
    echo_request.identifier(get_identifier());
    echo_request.sequence_number(++sequence_number_);
    compute_checksum(echo_request, body.begin(), body.end());

    // Encode the request packet.
    boost::asio::streambuf request_buffer;
    std::ostream os(&request_buffer);
    os << echo_request << body;

    // Send the request.
    time_sent_ = posix_time::microsec_clock::universal_time();
	using boost::asio::ip::tcp;    

	boost::asio::io_service io_service;
	tcp::resolver resolver(io_service);
	tcp::resolver::query query(boost::asio::ip::host_name(), "");
	tcp::resolver::iterator iter = resolver.resolve(query);
	tcp::resolver::iterator end; // End marker.
	while (iter != end)
	{
		tcp::endpoint ep = *iter++;
		if (ep.address().is_v4())
		{
			std::string ip;
			ip = ep.address().to_string();
			ip.erase(ip.rfind(".")+1);
			for (int count = 1; count < 255; count++)
			{
				std::ostringstream oss;
				oss << ip << count;
				icmp::resolver::query query(icmp::v4(), oss.str(), "");
				destination_ = *resolver_.resolve(query);
				socket_.send_to(request_buffer.data(), destination_);
				requests_++;
			}
		}
	}
	
    // Wait up to five seconds for a reply.
    num_replies_ = 0;
    timer_.expires_at(time_sent_ + posix_time::seconds(5));
    timer_.async_wait(boost::bind(&pinger::handle_timeout, this));
  }

  void handle_timeout()
  {
    if (num_replies_ == 0)
      std::cout << "Request timed out" << std::endl;

	requests_--;
    // Requests must be sent no less than one second apart.
    //timer_.expires_at(time_sent_ + posix_time::seconds(1));
    //timer_.async_wait(boost::bind(&pinger::start_send, this));
  }

  void start_receive()
  {
    // Discard any data already in the buffer.
    reply_buffer_.consume(reply_buffer_.size());

    // Wait for a reply. We prepare the buffer to receive up to 64KB.
    socket_.async_receive(reply_buffer_.prepare(65536),
        boost::bind(&pinger::handle_receive, this, _2));
  }

  void handle_receive(std::size_t length)
  {
    // The actual number of bytes received is committed to the buffer so that we
    // can extract it using a std::istream object.
    reply_buffer_.commit(length);

    // Decode the reply packet.
    std::istream is(&reply_buffer_);
    ipv4_header ipv4_hdr;
    icmp_header icmp_hdr;
    is >> ipv4_hdr >> icmp_hdr;

    // We can receive all ICMP packets received by the host, so we need to
    // filter out only the echo replies that match the our identifier and
    // expected sequence number.
    if (is && icmp_hdr.type() == icmp_header::echo_reply
          && icmp_hdr.identifier() == get_identifier()
          && icmp_hdr.sequence_number() == sequence_number_)
    {
      // If this is the first reply, interrupt the five second timeout.
      if (num_replies_++ == 0)
        timer_.cancel();

	  //https://192.168.0.213/spreedbox-notify/api/v1/static/scripts/spreedbox-notify.js
	  clients_.push_back(new client(io_service, ipv4_hdr.source_address().to_string().c_str(), "/spreedbox-notify/api/v1/static/scripts/spreedbox-notify.js"));
	  
      // Print out some information about the reply packet.
      posix_time::ptime now = posix_time::microsec_clock::universal_time();
      std::cout << length - ipv4_hdr.header_length()
        << " bytes from " << ipv4_hdr.source_address()
        << ": icmp_seq=" << icmp_hdr.sequence_number()
        << ", ttl=" << ipv4_hdr.time_to_live()
        << ", time=" << (now - time_sent_).total_milliseconds() << " ms"
        << std::endl;
    }

	requests_--;

	if (requests_ != 0)
	{
	    start_receive();
	}
  }

  static unsigned short get_identifier()
  {
#if defined(BOOST_WINDOWS)
    return static_cast<unsigned short>(::GetCurrentProcessId());
#else
    return static_cast<unsigned short>(::getpid());
#endif
  }

  icmp::resolver resolver_;
  icmp::endpoint destination_;
  icmp::socket socket_;
  deadline_timer timer_;
  unsigned short sequence_number_;
  posix_time::ptime time_sent_;
  boost::asio::streambuf reply_buffer_;
  std::size_t num_replies_;
  static int requests_;
  std::vector<client*> clients_;
};


void expiration_handler(boost::system::error_code ec)
{
	if (ec != boost::asio::error::operation_aborted)
        io_service.stop();
}

int pinger::requests_ = 0;

int main(int argc, char* argv[])
{
  try
  {
    int wait_time = 60;
	if (argc == 2) 
	{
		wait_time = atoi(argv[1]);
	}

	char* ip = "";
    pinger p(io_service, ip);
	timer.expires_from_now(posix_time::seconds(wait_time));
    timer.async_wait(expiration_handler);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}

