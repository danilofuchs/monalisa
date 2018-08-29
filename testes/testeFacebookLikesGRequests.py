import json
import grequests
import time
block= True
start = time.time()
def get_page_data(page_id,access_token):
    api_endpoint = "https://graph.facebook.com/v2.4/"
    fb_graph_url = api_endpoint+page_id+"?fields=id,name,link,fan_count&access_token="+access_token
    req = grequests.get(fb_graph_url, hooks=dict(response=print_response))
    request = req.send()

def print_response(response, *args, **kwargs) :
    try:
        page_data = json.loads(response.content)
    except (ValueError, KeyError, TypeError):
        return "JSON error"
    
    print("Page Name:"+ page_data['name'])
    print("Likes:"+ str(page_data['fan_count']))
    print("Link:"+ page_data['link'])
    block = False

page_id = "crossbotsutfpr" # username or id
token = "EAAfs9OJXRZB8BAAyYCgELf2sOZC46zp7ZAoQ7dOcTzO4wLYQ6UVK8hmAMzAsZBCG2fEZAmHFKKkMHXlmO6ACjR1O5TowZAPZBqc187D38TjQKKiCkRmFBTehPYuwXgeZByMySH0s0qc3U0ZBRvRyQ5TGOB2DNzLMYUM25SAOmUPT2ZBgZDZD"  # Access Token
page_data = get_page_data(page_id,token)

while block:
    a = 0
    if time.time() - start < 20 :
        break
    