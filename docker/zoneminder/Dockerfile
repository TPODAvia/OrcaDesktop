FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Moscow

# Enable all main repos (incl. universe) and update
RUN sed -i 's/^# deb/deb/' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    apt-get install -y \
        apache2 \
        mariadb-server \
        php php-mysql php-gd php-xml php-mbstring php-curl php-zip php-bcmath php-json php-cli \
        libapache2-mod-php \
        ffmpeg \
        zoneminder && \
    a2enmod rewrite headers expires cgi && \
    a2enconf zoneminder && \
    rm -rf /var/lib/apt/lists/*

# Init DB once at build time
RUN mkdir -p /var/run/mysqld && chown mysql:mysql /var/run/mysqld && \
    mysqld_safe --skip-networking & \
    sleep 10 && \
    mysql -e "CREATE DATABASE IF NOT EXISTS zm; GRANT ALL ON zm.* TO 'zmuser'@'localhost' IDENTIFIED BY 'zmpass'; FLUSH PRIVILEGES;" && \
    mysql zm < /usr/share/zoneminder/db/zm_create.sql && \
    mysql -e "SHUTDOWN;"

# Cache + logs permissions
RUN mkdir -p /var/cache/zoneminder /var/log/zm && \
    chown -R www-data:www-data /var/cache/zoneminder /var/log/zm && \
    # Clean any existing DB config lines to avoid duplicates
    sed -i '/^ZM_DB_HOST/d;/^ZM_DB_NAME/d;/^ZM_DB_USER/d;/^ZM_DB_PASS/d' /etc/zm/zm.conf && \
    echo "ZM_DB_HOST=localhost" >> /etc/zm/zm.conf && \
    echo "ZM_DB_NAME=zm"        >> /etc/zm/zm.conf && \
    echo "ZM_DB_USER=zmuser"    >> /etc/zm/zm.conf && \
    echo "ZM_DB_PASS=zmpass"    >> /etc/zm/zm.conf && \
    chown www-data:www-data /etc/zm/zm.conf && \
    chmod 640 /etc/zm/zm.conf


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 80

CMD ["/entrypoint.sh"]
