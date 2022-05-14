#!/usr/bin/env python
import requests
import json

class IFPS_image_poster:
    def __init__(self):

        self._api_url = 'https://api.nft.storage/upload/'
        self._headers = {
            'Authorization': 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJkaWQ6ZXRocjoweDE0ZDFjYzcyOTJlZmYzMTU5YjQ1MUQ1ZURDRkJFMUJBMkVBN2QzMTkiLCJpc3MiOiJuZnQtc3RvcmFnZSIsImlhdCI6MTY1MjUzMjg3MzA0NCwibmFtZSI6IkNoYWlubGlua1JvYm90In0.EFZXqd1hOicty_qpWo-yv5lZgsC0J6HKC9aCJKF-Pek',
            'Content-Type': 'image/jpg'
            }

    def post_and_receive_cid(self, image_path):

        with open(image_path, 'rb') as f:
            data = f.read()

        res = requests.post(url=self._api_url,
                            data=data,
                            headers=self._headers)
        cid = str(json.loads(res.text)['value']['cid'])

        print('Image is available at : https://' + cid + '.ipfs.nftstorage.link')

        return cid

if __name__ == '__main__':
    image_path = '/home/gepopa/Pictures/test.jpg';
    ifps_poster = IFPS_image_poster()
    cid = ifps_poster.post_and_receive_cid(image_path)
    print("Received CID: ", cid)
