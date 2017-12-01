#pragma once
#ifdef _WINDOWS_COMPILE_
#ifndef  ASR_API
#define ASR_API _declspec(dllexport)
#endif
#else
#define ASR_API
#endif

#include <map>
#include <iostream>
#include <memory>
#include <functional>

class RealTimeNlsRequest;

class nlsClient;
class ASR_API Nls
{
public:
	Nls(std::string url,int timeout = 20);
	~Nls();
	std::string _url;
	int _timeout;
	bool startNls();
	int sendAsr(char* buf, int Length);
	int sendOpus(char* buf, int Length);
	bool closeNls();
	bool setUserId(std::string id);
	bool setVocaId(std::string id);
	bool setSampleRate(int rate);
	bool setResponseMode(int mode);
	bool setFormat(std::string format);
	bool setApp_key(std::string key);
	bool authorize(std::string id, std::string scret);
	std::function<void(std::string data)> _onResultReceivedEvent = nullptr;	
private:
	std::shared_ptr<nlsClient> _client = nullptr;
	std::shared_ptr<RealTimeNlsRequest> _nlsRequest = nullptr;
};

