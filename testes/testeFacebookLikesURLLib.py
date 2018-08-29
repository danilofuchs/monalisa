import urllib.request as urllib2
import json

def get_page_data(page_id,access_token):
    api_endpoint = "https://graph.facebook.com/v2.4/"
    fb_graph_url = api_endpoint+page_id+"?fields=id,name,link,fan_count&access_token="+access_token
    try:
        api_request = urllib2.Request(fb_graph_url)
        api_response = urllib2.urlopen(api_request)
        
        try:
            return json.loads(api_response.read())
        except (ValueError, KeyError, TypeError):
            return "JSON error"

    except IOError as e:
        if hasattr(e, 'code'):
            return e.code
        elif hasattr(e, 'reason'):
            return e.reason

page_id = "crossbotsutfpr" # username or id
token = "EAAfs9OJXRZB8BAJSaQPZCybZATzR1b2KKHK97yEcH3QrLB1ItnnEqX6FTPbEoEEH5ZBvTDnZCJRZABNn30eVWANFSdHfMVqLq4Ur2Q91N4Ofhx0KtQzAjGby0jAxHdDV5ZAYJ5HHwQ7BZA13Ek59l9nnvFOpNmZAUfQfFPt9MwI6F41h1DMbIbLV0WrvSa1MykSgZD"  # Access Token
page_data = get_page_data(page_id,token)

print( "Page Name:"+ page_data['name'])
print("Likes:"+ str(page_data['fan_count']))
print("Link:"+ page_data['link'])
