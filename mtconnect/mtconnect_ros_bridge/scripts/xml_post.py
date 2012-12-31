import sys, httplib, urllib2, urllib

def do_request(xml_data):
    url = 'http://localhost:5005'
    data = {'flushCache':'false',
            'queryinput':xml_data}
    data = urllib.urlencode(data)
    request = urllib2.Request(url, data)

    response = urllib2.urlopen(request)
    payload = response.read()

    print(payload)

#HOST = '0.0.0.0'
#API_URL = /your/api/url

#def do_request(xml_data):
#    """HTTP XML Post request"""
#    webservice = httplib.HTTPConnection('0.0.0.0', 5000)
#    webservice.putrequest("POST", "")
#    webservice.putheader("Host", HOST)
#    webservice.putheader("User-Agent","Python post")
#    webservice.putheader("Content-type", "text/xml; charset=\"UTF-8\"")
#    webservice.putheader("Content-length", "%d" % len(xml_data))
#    webservice.endheaders()
#    webservice.send(xml_data)
#    statuscode, statusmessage, header = webservice.getreply()
#    result = webservice.getfile().read()
#    print('STATUSCODE:%d\nSTATUSMESSAGE:%s\nHEADER:' % (statuscode, statusmessage, header))
#    print('RESULT -->', result)


#def do_request(xml_data):
#	"""HTTP XML Post request, by www.forceflow.be"""
#	#request = open(xml_location,"r").read()
#	webservice = httplib.HTTP(HOST)
#	webservice.putrequest("POST", API_URL)
#	webservice.putheader("Host", HOST)
#	webservice.putheader("User-Agent","Python post")
#	webservice.putheader("Content-type", "text/xml; charset=\"UTF-8\"")
#	webservice.putheader("Content-length", "%d" % len(xml_data))
#	webservice.endheaders()
#	webservice.send(xml_data)
#	statuscode, statusmessage, header = webservice.getreply()
#	result = webservice.getfile().read()
#        print statuscode, statusmessage, header
#        print result

#import sys, httplib
#from urllib import *
#
#conn = HTTPConnection('localhost', 5000)
#conn.request("GET", "/current")
#response = conn.getresponse()
#if response.status != 200:
#    print "Request failed: %s - %d" % (response.reason, response.status)
#    if response.status == 404:
#        print 'Page Not Found'
#    sys.exit(0)
#print "Page Found Successfully, Outputting Request Body"
#body = response.read()
#seq = process_xml(body)
#
#conn.request("GET", "/sample?interval=1000&count=1000&from=" + seq)
#response = conn.getresponse()

