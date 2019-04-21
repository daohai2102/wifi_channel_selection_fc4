#ifndef _SSH_SESSION_H_
#define _SSH_SESSION_H_

#include <libssh/libssh.h>
#include <exception>
#include <cstring>
#include <string>

class SshSession{
private:
	char privateKeyFile[256];
	char userName[256];
	char address[256];
	int port;
	ssh_session mySshSession;
	bool connected;
public:
	SshSession();
	~SshSession();
	std::string runCommand(const char *cmd);
	ssh_channel runCommandAsync(const char *cmd);
	std::string getChannelBuffer(ssh_channel channel);
	SshSession& setUserName(const char *name);
	SshSession& setAddress(const char *addr);
	SshSession& setPort(int p);
	SshSession& setVerbosity(int verbo);
	SshSession& setPrivateKeyFile(const char *filename);
	SshSession& connectSsh();
};

class SshConnectException : public std::exception {
private:
	char message[256];
public:
	SshConnectException(){
		strcpy(message, "Error connecting\0");
	}
	SshConnectException(const char *mess) {
		strcpy(message, "Error connecting: \0");
		strcpy(message + strlen(message), mess);
	};
	const char *what() const throw(){
		return message;
	}
};

class SshUserAuthException : public std::exception {
private:
	char message[256];
public:
	SshUserAuthException(){
		strcpy(message, "Error user authenticating\0");
	}
	SshUserAuthException(const char *mess){
		strcpy(message, "Error user authenticating: \0");
		strcpy(message + strlen(message), mess);
	}
	const char *what() const throw(){
		return message;
	}
};

class SshNewChannelException : public std::exception {
private:
	char message[256];
public:
	SshNewChannelException(const char *mess){
		strcpy(message, "Error creating ssh channel: \0");
		strcpy(message + strlen(message), mess);
	}
	const char *what() const throw(){
		return message;
	}
};

class SshChannelOpenSessionException : public std::exception{
private:
	char message[256];
public:
	SshChannelOpenSessionException(){
		strcpy(message, "Error ssh_channel_open_session\0");
	}
	SshChannelOpenSessionException(const char *mess){
		strcpy(message, "Error ssh_channel_open_session: \0");
		strcpy(message + strlen(message), mess);
	}
	const char *what() const throw(){
		return message;
	}
};

class SshExecCommandException : public std::exception{
private:
	char message[256];
public:
	SshExecCommandException(const char *mess){
		strcpy(message, "Error executing command: \0");
		strcpy(message + strlen(message), mess);
	}
	const char *what() const throw(){
		return message;
	}
};

class SshChannelReadException : public std::exception{
private:
	char message[256];
public:
	SshChannelReadException(const char *mess){
		strcpy(message, "Error reading ssh response: \0");
		strcpy(message + strlen(message), mess);
	}
	const char *what() const throw(){
		return message;
	}
};

#endif
