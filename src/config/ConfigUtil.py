from pathlib import Path
import json
import logging



class ConfigUtil():
    """
     Class to load the ConfigConst configuration file.  
    """

    def __init__(self):

        fileName = "ConfigConst.json"
        filePath = Path(__file__).parent / fileName

        self.parsedJson = self._parseJson(filePath)

    def getValue(self, section: str, key: str):
        """ 
            This method accepts string params and 
            returns the value from the specific section of the json.

            @param section The section from which the value needs to be retrieved
            @param key The specific key from the section
            @return value from the section
        """
        return self.parsedJson.get(section, None).get(key, None)

    def _parseJson(self, filePath: str):
        """
        This method Deserializes the JSON file from the path provided
        
        @param filePath The path for the specified file.
        @return Deserialized json file.
        """

        try:
            with open(filePath, 'r') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            logging.error("Failed to load Config from the path specified")
        except:
            logging.error("An unexpected error has occuered while loading Config file")

        return None

# if __name__ == "__main__":
#     config = ConfigUtil().getValue("info", "IP")
#     print(config)