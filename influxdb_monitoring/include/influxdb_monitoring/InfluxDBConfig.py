
# Import necessary modules
import influxdb_client
from influxdb_client.client.write_api import SYNCHRONOUS
import json
import yaml

# Set default global variables
token_g = "MYTOKEN"
org_g = "myOrg"
url_g = "http://127.0.0.1:8086"
bucket_g="webdata"

# Create InfluxDB client and write API instances
client = influxdb_client.InfluxDBClient(url=url_g, token=token_g, org=org_g)
write_api = client.write_api(write_options=SYNCHRONOUS)

# Define a function to set InfluxDB configuration parameters
def setConfig(filename=None, token=None, org=None, url=None, bucket=None, docker_compose_filename=None) -> None:
    """
    Sets the global configuration variables for connecting to InfluxDB.

    filename: string
        Path to a JSON file containing the InfluxDB configuration parameters.
    token: string
        The InfluxDB authentication token.
    org: string
        The name of the InfluxDB organization.
    url: string
        The URL of the InfluxDB instance.
    bucket: string
        The name of the InfluxDB bucket.
    docker_compose_filename: string
        Path to a Docker Compose file containing the InfluxDB configuration parameters.
    """
    global token_g, org_g, url_g, bucket_g, client, write_api
    if not docker_compose_filename is None:
        print("Parsing docker compose file for InfluxDB settings. Using url=\"http://127.0.0.1:8086\" if no url is specified.")
        #print("Filename is: %s" %docker_compose_filename)
        with open(docker_compose_filename) as file:
            data = yaml.load(file, Loader=yaml.FullLoader)

        influxdb_config = data['services']['influxdb2']['environment']
        influxdb_token = None
        influxdb_org = None
        influxdb_bucket = None

        for item in influxdb_config:
            if item.startswith('DOCKER_INFLUXDB_INIT_ADMIN_TOKEN='):
                influxdb_token = item.split('=')[1]
            if item.startswith('DOCKER_INFLUXDB_INIT_ORG='):
                influxdb_org = item.split('=')[1]
            if item.startswith('DOCKER_INFLUXDB_INIT_BUCKET='):
                influxdb_bucket = item.split('=')[1]
        
        if influxdb_token is None or influxdb_org is None or influxdb_bucket is None:
            print("Could not find expected parameters in yaml. See example config or use another method.")
            raise Exception
        try:
            token_g = influxdb_token
            org_g = influxdb_org
            bucket_g = influxdb_bucket
            if url is None:
                url_g = "http://127.0.0.1:8086"
            else:
                url_g = url
        except:
            print("Could not read influxdb config from file %s" % filename)
            raise FileNotFoundError
    elif not filename is None:
        print("Parsing file for InfluxDB settings.")
        with open(filename) as json_file:
            data = json.load(json_file)
        try:
            token_g = data["token"]
            org_g = data["organization"]
            url_g = data["url"]
            bucket_g = data["bucket"]
        except:
            print("Could not read influxdb config from file %s" % filename)
            raise FileNotFoundError
    elif (not token is None and not org is None and not url is None and not bucket is None):
        print("Using function parameters for InfluxDB settings.")
        token_g = token
        org_g = org
        url_g = url
        bucket_g = bucket
    else:
        print("No valid input is provided.")
        raise Exception
    client = influxdb_client.InfluxDBClient(url=url_g, token=token_g, org=org_g)
    write_api = client.write_api(write_options=SYNCHRONOUS)
        
def printConfig():
    print(client.url)
    print(client.org)
    print(client.token)
    print(bucket_g)