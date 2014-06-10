var http=require("http");
var fs=require("fs");
var url=require("url");

var dgram=require("dgram");
var server=dgram.createSocket("udp4");

var PORT=2390;
var HOST="192.168.1.93";

var client=dgram.createSocket("udp4");

http.createServer(function(request,response){
	request.on("end",function(){
		var reqUrl=url.parse(request.url,true);
		var path=reqUrl.pathname;
		var _get=reqUrl.query;
		var page=getPage(path);
		var music=_get["music"];
		//var reqHeader=request.get("Content-Type");
		//console.log(reqHeader);
		//console.log("music: "+music);
		//console.log(path);
		if(music){
			sendMsg(music);
		}
		fs.readFile(page,"utf-8",function(error,data){
			if(error){
				response.writeHead(404);
				response.end();
			}else{
				response.writeHead(200,{
					'Content-Type':'text/html'
				});
				response.end(data);
			}
		});
	});
}).listen(80);

function getPage(path){
	var res=".";
	if(path=="/"){
		res="index.html";
	}else{
		res=res+path;
	}
	return res;
}
function sendMsg(text){
	var message=new Buffer(text);
	client.send(message, 0, message.length, PORT, HOST, function(err, bytes) {
		if (err) throw err;
		console.log('UDP message sent to ' + HOST +':'+ PORT);
	});
}