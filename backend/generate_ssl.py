import datetime
from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import serialization
import ipaddress
import os

def generar_certificados():
    dir_path = os.path.dirname(os.path.abspath(__file__))
    cert_file = os.path.join(dir_path, "cert.pem")
    key_file = os.path.join(dir_path, "key.pem")

    if os.path.exists(cert_file) and os.path.exists(key_file):
        print("Certificados ya existen.")
        return cert_file, key_file

    print("Generando clave privada RSA y certificado autofirmado...")
    key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=2048,
    )

    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "MX"),
        x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "Yucatan"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "TRAK Robotics"),
        x509.NameAttribute(NameOID.COMMON_NAME, "192.168.68.108"),
    ])

    cert = x509.CertificateBuilder().subject_name(
        subject
    ).issuer_name(
        issuer
    ).public_key(
        key.public_key()
    ).serial_number(
        x509.random_serial_number()
    ).not_valid_before(
        datetime.datetime.now(datetime.timezone.utc)
    ).not_valid_after(
        datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(days=365)
    ).add_extension(
        x509.SubjectAlternativeName([
            x509.DNSName("localhost"),
            x509.IPAddress(ipaddress.IPv4Address("192.168.68.108")),
            x509.IPAddress(ipaddress.IPv4Address("127.0.0.1")),
        ]),
        critical=False,
    ).sign(key, hashes.SHA256())

    with open(key_file, "wb") as f:
        f.write(key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption(),
        ))

    with open(cert_file, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))

    print(f"Certificados creados en:\n {cert_file}\n {key_file}")
    return cert_file, key_file

if __name__ == "__main__":
    generar_certificados()
