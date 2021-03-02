
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en-US" lang="en-US">
<head>
   <title>   Internet of Things  </title>
   <meta http-equiv="content-type" content="text/html; charset=iso-8859-1" />
</head>

   <body>



<?php
$servername = "localhost";
$username = "-";
$password = "-";

try {
    
       $conn = new PDO("mysql:host=$servername;dbname=iotRETO", $username, $password, array(PDO::MYSQL_ATTR_INIT_COMMAND => 'SET NAMES utf8'));
       $conn->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
       echo "<br>";
    }
catch(PDOException $e)
    {
       echo "Connection failed: " . $e->getMessage();
    }



$oxygen = $_POST["oxygen"];
$heart_rate = $_POST["heart_rate"];
$temperature = $_POST["temperature"];


$stmt = $conn->prepare("UPDATE datos_biometricos SET saturacion_oxigeno = '$oxygen', ritmo_cardiaco = '$heart_rate', temperatura = '$temperature' WHERE ID_medicion = (SELECT ID_medicion FROM datos_biometricos ORDER BY ID_medicion DESC LIMIT 0, 1)");

    
$stmt->execute();
$conn = null;


echo "Data submitted correctly.";
?>

</body>

</html>
