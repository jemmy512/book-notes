* Enable proxy for Shell and Git
    ```s
    # ~/.ssh/config
    Host github.com
        HostName github.com
        User git
        # HTTP Proxy
        # ProxyCommand socat - PROXY:127.0.0.1:%h:%p,proxyport=8080
        # socks5 (E.g., Shadowsocks0
        ProxyCommand nc -v -x 127.0.0.1:7890 %h %p
    ```

    ```s
    # ~/.gitconfig
    [http]
        proxy = socks5://127.0.0.1:7890
    [https]
        proxy = socks5://127.0.0.1:7890
    ```

* Python
    ```s
    # Do I have a Python 2 problem?
    $ python --version
    Python 2.7.10 # Referencing OSX system install

    $ which python
    /usr/bin/python # Yup, homebrew's would be in /usr/local/bin

    # Symlink /usr/local/bin/python to python3
    $ ln -s /usr/local/bin/python3 /usr/local/bin/python

    $ python --version
    Python 3.6.4 # Success!
    # If you still see 2.7 ensure in PATH /usr/local/bin/ takes precedence over /usr/bin/

    ln -s /usr/bin/python /usr/local/bin/python2
    ln -s /Users/jemmy/Library/Python/2.7/bin/pip /usr/local/bin/pip
    ```