# ── 代理地址 ────────────────────────────────────────
_http_proxy=http://10.225.138.117:7890
_socks_proxy=socks5://10.225.138.117:7890

# ── 标准环境变量 ────────────────────────────────────
export http_proxy=$_http_proxy
export https_proxy=$_http_proxy
export HTTP_PROXY=$_http_proxy
export HTTPS_PROXY=$_http_proxy

# ── SOCKS5 注释掉：会卡 git clone 等工具 ──────────
# export all_proxy=$_socks_proxy
# export ALL_PROXY=$_socks_proxy

# ── 清空不走代理的名单，确保强制走 ────────────────
export no_proxy=""
export NO_PROXY=""

# ── FTP ─────────────────────────────────────────────
export ftp_proxy=$_http_proxy
export FTP_PROXY=$_http_proxy

# ── rsync ───────────────────────────────────────────
export rsync_proxy=$_http_proxy
export RSYNC_PROXY=$_http_proxy

# ── curl ────────────────────────────────────────────
export CURL_PROXY=$_http_proxy

# ── wget ────────────────────────────────────────────
# wget 读 http_proxy / https_proxy / ftp_proxy，上面已有

# ── npm / yarn / node ──────────────────────────────
export npm_config_proxy=$_http_proxy
export npm_config_https_proxy=$_http_proxy
export yarn_proxy=$_http_proxy
export NODE_EXTRA_CA_CERTS=""  # 防止证书问题阻断

# ── Python pip ─────────────────────────────────────
# pip 读 http_proxy / https_proxy，上面已有

# ── Java / Gradle / Maven ───────────────────────────
export JAVA_TOOL_OPTIONS="-Dhttp.proxyHost=10.225.138.117 -Dhttp.proxyPort=7890 -Dhttps.proxyHost=10.225.138.117 -Dhttps.proxyPort=7890"

# ── Git wrapper：当前 shell 所有 git 命令自动走代理 ──
git() {
    GIT_CURL_VERBOSE=1 command git -c http.proxy=http://10.225.138.117:7890 "$@"
}

echo "  Proxy: ON  (当前 shell)"
echo "  http_proxy:  $_http_proxy"
echo "  all_proxy:   $_socks_proxy"