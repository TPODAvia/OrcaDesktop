#!/bin/bash

# Start MariaDB
mkdir -p /var/run/mysqld
chown mysql:mysql /var/run/mysqld
mysqld_safe &

# Wait DB
sleep 10

# Start Apache
apachectl start

# Start ZoneMinder
/usr/bin/zmpkg.pl start

echo "ZoneMinder started. Logs:"
touch /var/log/zm/zm_debug.log
exec tail -F /var/log/zm/*.log
