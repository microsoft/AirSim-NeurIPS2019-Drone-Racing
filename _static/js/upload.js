

const uploadButton = document.getElementById("upload-button");
const tier1File = document.getElementById("tier1-file");
const tier2File = document.getElementById("tier2-file");
const tier3File = document.getElementById("tier3-file");
const reportFile = document.getElementById("report-file");
// const submitForm = document.getElementById("submit-form");

const reportStatus = message => {
    status.innerHTML += `${message}<br/>`;
    status.scrollTop = status.scrollHeight;
}

const accountName = "gameofdrones";
const sasString = "se=2020-12-31&sp=rwdlac&sv=2018-03-28&ss=b&srt=sco&sig=DeDJZj%2Bu2n0u2NifyFX/lQbXxJam4TOARLupjVqpYCs%3D";
const containerName = "uploads";
const containerURL = new azblob.ContainerURL(
    `https://${accountName}.blob.core.windows.net/${containerName}?${sasString}`,
    azblob.StorageURL.newPipeline(new azblob.AnonymousCredential));


    const createContainer = async () => {
    try {
        reportStatus(`Creating container "${containerName}"...`);
        await containerURL.create(azblob.Aborter.none);
        reportStatus(`Done.`);
    } catch (error) {
        reportStatus(error.body.message);
    }
};




const uploadFiles = async () => {
    try {
        var teamName = document.getElementById("team-name").value;
        var teamId = document.getElementById("team-id").value;
        // example upload time "2019-09-03T02:55:46.841Z"
        var uploadTime = new Date(Date.now()).toISOString().replace(":", ".")

        console.log("Upload Files");

        const promises = [];
        // Tier 1
        for (const file of tier1File.files) {
            console.log('file:' + file);
            const blockBlobURL = azblob.BlockBlobURL.fromContainerURL(
                containerURL, uploadTime + '-' + teamName + '-' + teamId + '-' + file.name);
            console.log('blockBlobURL:' + blockBlobURL)
            promises.push(azblob.uploadBrowserDataToBlockBlob(
                azblob.Aborter.none, file, blockBlobURL));
        }
        // Tier 2
        for (const file of tier2File.files) {
            console.log(file);
            const blockBlobURL = azblob.BlockBlobURL.fromContainerURL(
                containerURL, uploadTime + '-' + teamName + '-' + teamId + '-tier2-' + file.name);
            promises.push(azblob.uploadBrowserDataToBlockBlob(
                azblob.Aborter.none, file, blockBlobURL));
        }
        // Tier 3
        for (const file of tier3File.files) {
            console.log(file);
            const blockBlobURL = azblob.BlockBlobURL.fromContainerURL(
                containerURL, uploadTime + '-' + teamName + '-' + teamId + '-tier3-' + file.name);
            promises.push(azblob.uploadBrowserDataToBlockBlob(
                azblob.Aborter.none, file, blockBlobURL));
        }
        // Reports 
        for (const file of reportFile.files) {
            console.log(file);
            const blockBlobURL = azblob.BlockBlobURL.fromContainerURL(
                containerURL, uploadTime + '-' + teamName + '-' + teamId + '-report-' + file.name);
            promises.push(azblob.uploadBrowserDataToBlockBlob(
                azblob.Aborter.none, file, blockBlobURL));
        }


        await Promise.all(promises);
        alert("Upload complete");
        location.reload();
        // submitForm.reset()     

    } catch (error) {
        console.log(error.body.message);

    }
}

uploadButton.addEventListener("click", uploadFiles);



