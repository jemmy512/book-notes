# gerrit

```sh
repo init -u https://example.com/platform/manifest -b main
repo sync


repo start my-feature --all
# Edit files in repo1 and repo2
cd repo1 && git add . && git commit -m "Add feature X"
cd ../repo2 && git add . && git commit -m "Add feature Y"


repo upload --topic my-feature-topic

ssh -p 29418 user@gerrit.example.com gerrit query topic:my-feature-topic
```