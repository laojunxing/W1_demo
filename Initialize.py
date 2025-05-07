import json
import logging  

class Initialize:
    def __init__(self):
        pass
    @staticmethod
    def load_config(confile):
        with open(confile,  'r',encoding='utf-8') as f:
            configdata = json.load(f)
        language_data = configdata['Language']
        selected_language = language_data['Language']
        log_msg = language_data[selected_language]
        if log_msg["load config"][1]==1: logging.info(log_msg["load config"][0].format(confile))  
        return configdata  ,log_msg